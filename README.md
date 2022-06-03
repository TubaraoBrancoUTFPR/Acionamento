# Acionamento
	Project:		Veículo elétrico (Equipe Tubarão Branco).

	Description: 	Acionamento (a 120 graus) do Motor BLDC (com driver IR2130).
					UTFPR Pato Branco.

 	Device(s):		STM32F103 (versão 9); STM32F407 (até versão 8)

  	Old Version control: 
  	[Ref#]	- [Date]	 - [Author]		- [Detail]
 		v1	- 2019-11-22 - Prof.Torrico	- (nome BLDC120) Versao original: acionamento a 120 graus, sentido horario.
 		v2	- 2019-11-22 - Prof.Vargas	- (renomeado STM32F4-BLDC) Logica acionamento corrigida (para: ativo em baixo, IR2130).
 										- Testado com motor a vazio e fontes de bancada: tudo Ok.
 		v3	- 2019-11-23 - Prof.Vargas	- Adicionada flag no botao liga.
 										- Adicionado Comentarios e Documentacao.
 		v4	- 2019-11-27 - Prof.Vargas	- Sentido de rotacao corrigido (para: anti-horario).
 										- Adicionado leitura do FAULT do IR2130.
 										- Removido flag do botao.
 		v5	- 2019-11-28 - Prof.Vargas	- Adicionada rampa nos PWMs com valor maximo configuravel pelo ADC
 		v6	- 2020-02-06 - Prof.Vargas	- (renomeado STM32F407-BLDC)Importado no Stm32CubeIDE v1.2.1 (originalmente era no Atollic TrueStudio v9.3.0)
		v7	- 2020-02-13 - IHM #1:    	- (renomeado CAN_STM32F407-BLDC)Adicionado Rede CAN, Implementado safety timer;
		#1: Caio Aderne, Eduardo Junior, Gabriel Alessandro, Gustavo de Souza, Julia Balbinotti, Laisa Langhinoti, Stefanie Oliveira.
		v8	- 2020-02-17 - Eduardo Jr.	- Chave ligada ao PB11 alterna entre CAN e Potenciometro para alterar o duty cycle;
										- Frequencia de chaveamento alterada para 1kHz (prescaler 40+1), se fosse 5,1kHz seria pre=7+1, e era 10,2kHz com pre=3+1.
		v9	- 2020-02-17 - Prof.Torrico - Código portado para o STM32F103.
										Baseado no CAN_STMF407-BLDC_v7, mas foi removido recebimento do duty por rede CAN
										Frequência de chaveamento de 4,? kHz
		v10 							- (adicionado rede CAN, está com Mancuso)
		v11 - 2020-03-18 - Prof.Torrico	- Baseado na V9, então não tem a CAN da v10
										- Remapeado setores do motor (importante)
										- Na placa K4(c), feita na JLCPCB, temos 2 chaves PA1 e PA2
		v12 - 2021-10-12 - Prof.Vargas	- Baseada na v11, mas alterando para PA1 e PA2 mudar entre 3 modos de operação:
										recebe duty pelo potenciômetro, duty fixo e recebe pela rede CAN.
										-Adicionado o código da rede CAN da v10.
										-Adicionado limitação de duty cycle máximo
		v13 - 2021-10-12 - Eduardo Jr   -Ajuste na CAN
