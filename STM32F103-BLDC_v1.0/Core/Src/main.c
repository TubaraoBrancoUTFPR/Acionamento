/* USER CODE BEGIN Header */
/*
 * ******************************************************************************
*	Project:		Veículo elétrico (Equipe Tubarão Branco).
*
*	Description: 	Acionamento (a 120 graus) do Motor BLDC (com driver IR2130).
*					UTFPR Pato Branco.
*
* 	Device(s):		STM32F103 (versão 9); STM32F407 (até versão 8)
*
*  	Old Version control:
*  	[Ref#]	- [Date]	 - [Author]		- [Detail]
* 		v1	- 2019-11-22 - Prof.Torrico	- (nome BLDC120) Versao original: acionamento a 120 graus, sentido horario.
* 		v2	- 2019-11-22 - Prof.Vargas	- (renomeado STM32F4-BLDC) Logica acionamento corrigida (para: ativo em baixo, IR2130).
* 										- Testado com motor a vazio e fontes de bancada: tudo Ok.
* 		v3	- 2019-11-23 - Prof.Vargas	- Adicionada flag no botao liga.
* 										- Adicionado Comentarios e Documentacao.
* 		v4	- 2019-11-27 - Prof.Vargas	- Sentido de rotacao corrigido (para: anti-horario).
* 										- Adicionado leitura do FAULT do IR2130.
* 										- Removido flag do botao.
* 		v5	- 2019-11-28 - Prof.Vargas	- Adicionada rampa nos PWMs com valor maximo configuravel pelo ADC
* 		v6	- 2020-02-06 - Prof.Vargas	- (renomeado STM32F407-BLDC)Importado no Stm32CubeIDE v1.2.1 (originalmente era no Atollic TrueStudio v9.3.0)
*		v7	- 2020-02-13 - IHM #1:    	- (renomeado CAN_STM32F407-BLDC)Adicionado Rede CAN, Implementado safety timer;
*		#1: Caio Aderne, Eduardo Junior, Gabriel Alessandro, Gustavo de Souza, Julia Balbinotti, Laisa Langhinoti, Stefanie Oliveira.
*		v8	- 2020-02-17 - Eduardo Jr.	- Chave ligada ao PB11 alterna entre CAN e Potenciometro para alterar o duty cycle;
*										- Frequencia de chaveamento alterada para 1kHz (prescaler 40+1), se fosse 5,1kHz seria pre=7+1, e era 10,2kHz com pre=3+1.
*		v9	- 2020-02-17 - Prof.Torrico - Código portado para o STM32F103.
*										Baseado no CAN_STMF407-BLDC_v7, mas foi removido recebimento do duty por rede CAN
*										Frequência de chaveamento de 4,? kHz
*		v10 							- (adicionado rede CAN, está com Mancuso)
*		v11 - 2020-03-18 - Prof.Torrico	- Baseado na V9, então não tem a CAN da v10
*										- Remapeado setores do motor (*importante)
*										- Na placa K4(c), feita na JLCPCB, temos 2 chaves PA1 e PA2
*		v12 - 2021-10-12 - Prof.Vargas	- Baseada na v11, mas alterando para PA1 e PA2 mudar entre 3 modos de operação:
*										recebe duty pelo potenciômetro, duty fixo e recebe pela rede CAN.
*										-Adicionado o código da rede CAN da v10.
*										-Adicionado limitação de duty cycle máximo
*		v13 - 2021-10-12 - Eduardo Jr   -Ajuste na CAN
*
*
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define A_LOW_RESET 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET) //U_Low
#define	B_LOW_RESET		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET) //V_Low
#define	C_LOW_RESET 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET) //W_Low
#define A_LOW_SET 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET) //U_Low
#define	B_LOW_SET 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET) //V_Low
#define	C_LOW_SET 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET) //W_Low

#define LIGA_LED 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET) //LED_Board
#define	DESLIGA_LED 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET) //LED_Board
#define	PISCA_LED 		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef pTxHeader;		//Variavel do envio da can
CAN_RxHeaderTypeDef pRxHeader; 		//Variavel do recebimento da can
uint32_t pTxMailbox;				//Caixa de envio da can
uint8_t txData[8];					//Vetor de envio pela CAN
uint8_t rxData[8];					//Vetor de recebimento pela CAN
uint16_t adcRxData;					//Variavel para o duty
CAN_FilterTypeDef sFilterConfig;

				//Mensagem de recebimento da can, variaveis que entram na interrupçcao devem ser volateis
	//Recebimento da CAN acontece na função "USB_LP_CAN1_RX0_IRQHandler(void)" no arquivo "stm32f1xx_it.c"

CAN_FilterTypeDef sFilterConfig;	//Variavel do Filtro de recebimento

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t 	Hall_A, Hall_B, Hall_C; // estado dos sensores de efeito HALL
uint8_t 	setor=0, setor_a=0, setor_b=0, setor_c=0, setor_real=0, setor_ant;
uint16_t cont=0;

// Duty Cycle deve estar entre (0~4095)
	//Obs.: IR2130 tem logica invertida: quanto maior (no codigo) esse numero,  menor sera o duty (nas chaves).
	//0.1 -> 10% = 3685
	//0.2 -> 20% = 3276
	//0.3 -> 30% = 2866
	//0.4 -> 40% = 2457  ***estava aqui nos testes de 2021
	//0.5 -> 50% = 2047
	//0.8 -> 80% = 819
	//0.9 -> 90% = 409
#define	DutyMaximo	2047
#define adcVolanteMaximo 3400
volatile unsigned int DutyRedeCAN = 4095;	//Duty cycle recebido via rede CAN
volatile unsigned int DutyDesejado = 4095;	//Duty cycle



//A placa terá 4 modos de operação, de acordo com as 2 chaves azuis (S1=PA2 e S3=PA1)
#define 	MODO_DESLIGADO			0
#define 	MODO_POTENCIOMETRO		1
#define 	MODO_DUTY_FIXO			2
#define 	MODO_REDE_CAN			3
volatile uint8_t MODO_DE_OPERACAO = MODO_DESLIGADO;	//Armazena o modo operação da placa

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim1); //depois de inicializar habilita o timer 1
  //HAL_TIM_Base_Start_IT(&htim2);

  DESLIGA_LED;	//Desliga LED da board

//Configuraçao da rede CAN:
	//a) Configuraçao do pTxHeader (envio)
//		pTxHeader.DLC = 1;				//Quantidade de Pacotes a serem enviado
//		pTxHeader.IDE =	CAN_ID_STD;		// identificador se é 11bits(CAN_ID_STD) ou 29bits(CAN_ID_EXT)
//		pTxHeader.RTR = CAN_RTR_DATA;	// Se é REMOTE(CAN_RTR_REMOTE) ou DATA(CAN_RTR_DATA)
//		pTxHeader.StdId	= 0x244;		//Identificação desse nó da rede
										//ID Padrao 11bits, number between Min_Data = 0 and Max_Data = 0x7FF
//		//pTxHeader.ExtId 				//ID extendido 29bits,number between Min_Data = 0 and Max_Data = 0x1FFFFFFF


		//b) Configuraçcao do Filtro (recebimento)
		/*sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //Usar filtro 0 ou filtro 1 -> CAN_FILTER_FIFOx
		sFilterConfig.FilterIdHigh = 0x245<<5;		//Recebe dessa ID: number between Min_Data = 0x0000 and Max_Data = 0xFFFF
		sFilterConfig.FilterIdLow = 0;				//number between Min_Data = 0x0000 and Max_Data = 0xFFFF
		sFilterConfig.FilterMaskIdHigh = 0;			//number between Min_Data = 0x0000 and Max_Data = 0xFFFF
		sFilterConfig.FilterMaskIdLow = 0;			//number between Min_Data = 0x0000 and Max_Data = 0xFFFF
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//Dois filtros de 16(CAN_FILTERSCALE_16BIT) ou 1 de 32(CAN_FILTERSCALE_32BIT);
		sFilterConfig.FilterActivation = ENABLE; 	//Se ta ativado ou nao
		HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //Aplica as configuraçao do filtro

		//c) Inicia a rede
		if (HAL_CAN_Start(&hcan) != HAL_OK) // Começando a rede can
		{
			 Start Error
			 Error_Handler();
		}
		HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);//Ativando o recebimento da flag de interrupção na fila 0


*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  HAL_Delay(500); //milliseconds

//Lê as 2 chaves e escolhe modo de operação:
	  //Se a chave S3 = PA1 está desligada
	  // e a chave S1 = PA2 está desligada
	  // então recebe duty pela rede CAN
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)==0)
	  {
		  MODO_DE_OPERACAO = MODO_REDE_CAN;
	  }
	  //Se a chave S3 = PA1 está ligada
	  // e a chave S1 = PA2 está desligada
	  // então duty pelo potenciômetro
	  else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)==0)
	  {
		  MODO_DE_OPERACAO = MODO_POTENCIOMETRO;
	  }
	  //Se a chave S3 = PA1 está ligada
	  // e a chave S1 = PA2 está ligada
	  // então duty fica fixo
	  else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)==1)
	  {
		  MODO_DE_OPERACAO = MODO_DUTY_FIXO;
	  }
	  // Se for outra combinação (no caso, sobra S3=on; S1=off)
	  // então desliga o inversor
	  else
	  {
		  MODO_DE_OPERACAO = MODO_DESLIGADO;
	  }

//	  MODO_DE_OPERACAO = MODO_REDE_CAN; //apagar essa lsinha e descomentar 220-247

//APLICA OS MODOS DE OPERAÇÃO
  	switch(MODO_DE_OPERACAO)
		{
  			//DUTY DO INVERSOR CONTROLADO PELO POTENCIÔMETRO
  			case(MODO_POTENCIOMETRO):
			{
				  //Converter ADC
				  HAL_ADC_Start(&hadc1);
				  HAL_ADC_PollForConversion(&hadc1, 100);		//ADC, (potenciometro)
				  DutyDesejado = 4095 - (HAL_ADC_GetValue(&hadc1)>>1);
				  	  	  	  //ADC com 3,3V = 4030
				  	  	  	  //ADC com 0V = 0
				  	  	  	  //PWM 4095 = 0%; 0 =100%
				  	  	  	  //4095 - (4030/2) = 2.080  (aprox.50%)
				  HAL_ADC_Stop(&hadc1);
  				 // DutyDesejado= 3000;
				  if (DutyDesejado < DutyMaximo)
				  {
					  DutyDesejado = DutyMaximo;
				  }
				  break;
			}

	  		//DUTY DO INVERSOR FIXO
			case(MODO_DUTY_FIXO):
			{
				  //DutyDesejado = 2047; //4095 * 50% = 2047 (para duty de 50%)
				  //DutyDesejado = 3685; //4095 * 90% = 3685 (para duty de 10%, pois é negado)
				  DutyDesejado = 3890; //4095 * 95% = 3890 (para duty de 5%, pois é negado)
			      //DutyDesejado = 4000; //4000 =~2,32%
				  //DutyDesejado = 1228; //1228 =~70%
				  //DutyDesejado = DutyMaximo;
				  break;
			}


			case(MODO_REDE_CAN):
			{
				  DutyDesejado = DutyRedeCAN;	//Pega o Duty pela rede CAN

				  if (DutyDesejado < DutyMaximo)
				  {
					  DutyDesejado = DutyMaximo;
				  }
				  break;
			}

			case(MODO_DESLIGADO):
			{
				DutyDesejado = 4095;	//4095 = Com isso desliga o PWM
				break;
			}
		}





  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  	pTxHeader.DLC = 2;
  	pTxHeader.IDE = CAN_ID_STD;
  	pTxHeader.RTR = CAN_RTR_DATA;
  	pTxHeader.StdId = 0x244;

  	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  	sFilterConfig.FilterIdHigh = 0x245 << 4;
  	sFilterConfig.FilterIdLow = 0;
  	sFilterConfig.FilterMaskIdHigh = 0;
  	sFilterConfig.FilterMaskIdLow =0;
  	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  	sFilterConfig.FilterActivation = ENABLE;

  	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  	if (HAL_CAN_Start(&hcan) != HAL_OK)
  	{
  		/* Start Error */
  		Error_Handler();
  	}
  	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, U_LOW_Pin|V_LOW_Pin|W_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Board_Pin */
  GPIO_InitStruct.Pin = LED_Board_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Board_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIGA_DESLIGA_Pin PA2 */
  GPIO_InitStruct.Pin = LIGA_DESLIGA_Pin|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_W_Pin HALL_V_Pin HALL_U_Pin */
  GPIO_InitStruct.Pin = HALL_W_Pin|HALL_V_Pin|HALL_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : U_LOW_Pin V_LOW_Pin W_LOW_Pin */
  GPIO_InitStruct.Pin = U_LOW_Pin|V_LOW_Pin|W_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//tratamento da interrupcao do timer 1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//void PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
	{
		//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000); // mudar o Duty Cycle (configurado de 0-4095)
		//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000); // mudar o Duty Cycle (configurado de 0-4095)
		//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2000); // mudar o Duty Cycle (configurado de 0-4095)

		// Leitura dos sensores de efeito Hall
		Hall_A=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
		Hall_B=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
		Hall_C=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);


	// emualador dos sensores de efeito Hall


//	   if(cont<4500)
//	   {
//		   cont++;
//	   }
//	   else
//	   {
//		   cont=0;
//	   }
//
//	   switch (cont)
//	   {
//	   case(0):
//			   Hall_A=1;
//			   break;
//	   case(750):
//				Hall_C=0;
//			   break;
//	   case(1500):
//				Hall_B=1;
//			   break;
//	   case(2250):
//				Hall_A=0;
//			   break;
//	   case(3000):
//				Hall_C=1;
//			   break;
//	   case(3750):
//				Hall_B=0;
//			   break;
//	   }

	   // fim do emulador do sensor de efeito hall

		//Observa qual o setor indicado pelos sensores Hall de posicao do motor:
	    if (Hall_A==0 && Hall_B==1 && Hall_C==0) 	//setor 1
	    {
	    	setor=1;
	    }
	    else if (Hall_A==0 && Hall_B==1 && Hall_C==1) //setor 2
	    {
	    	setor=2;
	    }
	    else if (Hall_A==0 && Hall_B==0 && Hall_C==1) //setor 3
	    {
	    	setor=3;
	    }
	    else if (Hall_A==1 && Hall_B==0 && Hall_C==1) //setor 4
	    {
	    	setor=4;
	    }
	    else if (Hall_A==1 && Hall_B==0 && Hall_C==0) //setor 5
	    {
	    	setor=5;
	    }
	    else if (Hall_A==1 && Hall_B==1 && Hall_C==0) //setor 6
	    {
	    	setor=6;
	    }
	    else
	    {
	    	setor=0; //setor inexistente, desligar todas as chaves
	    }


	    //"Filtro" dos sensores
	    //Confere se as 3 �ltimas indica��es dos sensores Hall s�o iguais:
	    setor_c=setor_b;
	    setor_b=setor_a;
	    setor_a=setor;


	    //Se obtiver 2 detecções de setores iguais:
	    if(setor_a==setor_b)
	    {
	    	//Desliga as chaves high (IR2130 ativo em baixo, entao para deligar tem que ativar as sa�da)
	    	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4095); //Chave U_High
	    	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4095); //Chave V_High
	    	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,4095); //Chave W_High
	    }

	    //Se obtiver 3 detecções de setores iguais:
	    if(setor_a==setor_b && setor_b==setor_c) // se repetir numa janela de tres amostras seguidas (talvez pegar mais)
	    {
	    	setor_ant=setor_real;
	    	setor_real=setor;
	    	//poderia montar uma logica de sequencia de setores (para nao pular setor)


	    	//Chaveamento a partir do setor real
	    	//Nova logica (2020-03)
	    	switch(setor_real)
			{
				case(0):	//Desliga todas as chaves (IR2130 ativo em baixo, entao para deligar tem que ativar as saida)
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4095); //Chave U_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4095); //Chave V_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,4095); //Chave W_High
					A_LOW_SET; 		//Chave U_Low
					B_LOW_SET; 		//Chave V_Low
					C_LOW_SET; 		//Chave W_Low
				break;

				case(1):	// (IR2130 ativo em baixo)
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4095); //Chave U_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4095); //Chave V_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,DutyDesejado); //Chave W_High
					A_LOW_RESET; 	//Chave U_Low -> ligada
					B_LOW_SET; 		//Chave V_Low
					C_LOW_SET; 		//Chave W_Low
				break;

				case(2): // (IR2130 ativo em baixo)
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4095); //Chave U_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4095); //Chave V_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,DutyDesejado); //Chave W_High
					A_LOW_SET; 		//Chave U_Low
					B_LOW_RESET; 	//Chave V_Low -> ligada
					C_LOW_SET; 		//Chave W_Low
				break;

				case(3): // (IR2130 ativo em baixo)
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,DutyDesejado); //Chave U_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4095); //Chave V_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,4095); //Chave W_High
					A_LOW_SET; 		//Chave U_Low
					B_LOW_RESET; 	//Chave V_Low -> ligada
					C_LOW_SET; 		//Chave W_Low
				break;

				case(4): // (IR2130 ativo em baixo)
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,DutyDesejado); //Chave U_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4095); //Chave V_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,4095); //Chave W_High
					A_LOW_SET; 		//Chave U_Low
					B_LOW_SET; 		//Chave V_Low
					C_LOW_RESET; 	//Chave W_Low -> ligada
				break;

				case(5): // (IR2130 ativo em baixo)
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4095); //Chave U_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,DutyDesejado); //Chave V_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,4095); //Chave W_High
					A_LOW_SET; 		//Chave U_Low
					B_LOW_SET;		//Chave V_Low
					C_LOW_RESET; 	//Chave W_Low -> ligada

				break;

				case(6): // (IR2130 ativo em baixo)
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4095); //Chave U_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,DutyDesejado); //Chave V_High
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,4095); //Chave W_High
					A_LOW_RESET; 	//Chave U_Low -> ligada
					B_LOW_SET; 		//Chave V_Low
					C_LOW_SET; 		//Chave W_Low
				break;
			}
	    }

}


//Interrupcao da CAN

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, rxData);
	  adcRxData = (rxData[0] << 8) | rxData[1];
	  //DutyRedeCAN = (unsigned int)adcRxData;
	  DutyRedeCAN = ((((DutyMaximo-4095.0)/adcVolanteMaximo)*(unsigned int)adcRxData)) + 4095;
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
