/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* 
	============================ CONTROLADOR DE TEMPERATURA E UMIDADE =============================

	Resumo do Funcionamento:

    Controle de Irrigação
      Ativar: Se a umidade do solo (umid) for menor ou igual a umid_min.
      Desativar: Se a umidade do solo atingir umid_min + 75% do intervalo entre umid_min e umid_max.

    Controle de Refrigeração
      Ativar: Se a temperatura (temp) for maior ou igual a temp_max.
      Desativar: Se a temperatura cair abaixo de temp_max - 25% do intervalo entre temp_min e temp_max.

    Controle de Aquecimento
      Ativar: Se a temperatura for menor ou igual a temp_min.
      Desativar: Se a temperatura subir acima de temp_min + 25% do intervalo entre temp_min e temp_max.

    Controle do LED de Alerta
      Ativar: Se qualquer uma das seguintes condições for atendida:
        A temperatura estiver próxima de temp_min ou temp_max enquanto o aquecimento/refrigeração estiver desligado.
        A umidade do solo estiver próxima de umid_min enquanto a irrigação estiver desligada.

	Funções Principais:

		incrementoDecremento: A função incrementoDecremento permite incrementar ou decrementar o valor de uma variável com base em entradas de botões, atualizando o display correspondente em cada ciclo do loop. O loop continua até que o botão de função seja pressionado novamente para sair. Em cada iteração, o display é atualizado com o valor atual da variável dividida em dígitos individuais. A variável é incrementada ou decrementada dependendo se os botões de incremento ou decremento são pressionados, e há um pequeno atraso (HAL_Delay) para evitar múltiplas leituras rápidas dos botões. Quando o botão de função é pressionado novamente, o loop é encerrado e a variável é retornada com o valor atualizado.

		atualizarDisplay/registerx: A função atualizarDisplay atualiza o valor exibido em um display específico, chamando a função register1, register2 ou register3 com base no parâmetro display. A função register1 (similar às funções register2 e register3) envia o valor correspondente ao display em forma de bits através de um processo de deslocamento e armazenamento de dados. Programação desenvolvida para uso de displays cátodo comum em conjunto com registradores de deslocamento 74HC595.

		controlarSaidas: A função controlarSaidas monitora a umidade do solo (umid) e a temperatura (temp), controlando sistemas de irrigação, aquecimento e refrigeração com base em limiares pré-definidos. Além disso, ela ativa um LED de alerta se os valores dos sensores estiverem próximos aos limites críticos.

	Conexão do CI no display de 7 seguimentos:

		O CI possuí 8 saídas comuns e uma negada sendo conectado ao display da seguinte forma:
		8 -> A | 7 -> B | 6 -> C | 5 -> D |4 -> E | 3 -> F | 2 -> G
		-	A saída 1 pode ser conectada ao ponto e a saída negada não está sendo utilizada.
		-	A interpretação da função segue que a saída 8 é LSB e a saída 1 é MSB.

	Números em decimal inseridos na função register para apresentar algarismos no display:

    0 = 0111 1110 = 126   
    1 = 0000 1100 = 12    
    2 = 1011 0110 = 182   
    3 = 1001 1110 = 158
    4 = 1100 1100 = 204
    5 = 1101 1010 = 218
    6 = 1111 1010 = 250
    7 = 0000 1110 = 14
    8 = 1111 1110 = 254
    9 = 1101 1110 = 222

	Conversão de valores para aparecer nos displays:

		dez_temp = temp/10;
		uni_temp = temp%10;
		Através da divisão e do resto da divisão o valor da temperatura/umidade é dividido em unidade e dezena, sendo que a unidade vai para o display 3 e a dezena para o display 2.
	
	Ativação dos botões para controle:

		Funções:
			0 - Mostrador (configurar para voltar ao mostrador)
			1 - temp_min
			2 - temp_max
			3 - umid_min
			4 - umid_max
		Funcionamento:
      Clicar no botão de função.
      Vai aparecer 0 no primeiro botão, clique novamente e escolha o parâmetro que se deseja cadastrar.
      Botão 2 incrementa o valor e salva na variável.
      Botão 3 decrementa o valor e salva na variável.
      Em 0, ao precionar incremento ou decremento, a setagem de valores é encerrada e retorna o loop normal.
  
  Programação e configuração de timers, interrupções, PWM e ADC:

	  A programação é baseada em interrupções. Sempre que a função HAL_TIM_PeriodElapsedCallback é chamada (o que ocorre ao final de algum timer), uma rotina específica é executada.
    Timers
      TIM3: Responsável pelo PWM (Modulação por Largura de Pulso) com um ciclo de trabalho (duty cycle) de 50% e uma frequência de 10Hz. Sempre que os parâmetros chegam a uma margem de 10% próximo aos valores estabelecidos (limites críticos), o PWM é ativado, emitindo um sinal visual no LED de alerta.
      TIM4: Finaliza a cada 1 segundo, chamando a rotina de interrupção que realiza a leitura dos potenciômetros e a conversão AD (Analógica para Digital).
      TIM5: Finaliza a cada 1 segundo, chamando a rotina de interrupção que atualiza os valores no display. A cada 1 segundo, um valor (temperatura ou umidade) é apresentado alternadamente.
    ADC
      O ADC (Conversor Analógico-Digital) é utilizado para ler os valores dos potenciômetros que representam os sensores de temperatura e umidade.
    Loop WHILE
      O loop WHILE é responsável pelo cadastro de parâmetros. Quando o botão de função é pressionado, os timers (TIM4 e TIM5) são parados, e inicia-se o processo de configuração dos parâmetros. Ao final desse processo, os timers são reativados.
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define data1         GPIO_PIN_2 // Entrada de Dados
#define shift1        GPIO_PIN_3 // Registrador de Saída
#define store1        GPIO_PIN_4 // Registrador de Deslocamento

#define data2         GPIO_PIN_5 
#define shift2        GPIO_PIN_6  
#define store2        GPIO_PIN_7

#define data3         GPIO_PIN_0 //ATENÇÃO: Porta GPIOB
#define shift3        GPIO_PIN_1
#define store3        GPIO_PIN_2

#define sensor_umid   ADC_CHANNEL_0
#define sensor_temp   ADC_CHANNEL_1

#define irrigacaoA    GPIO_PIN_8
#define aquecimentoA  GPIO_PIN_11
#define refrigeracaoB GPIO_PIN_3

#define incremento    GPIO_PIN_12
#define decremento    GPIO_PIN_13
#define funcao        GPIO_PIN_15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
uint32_t analog_temp = 0, analog_umid = 0, temp = 0, umid = 0, atualizacao = 0, menu_opcao = 0, umid_min = 60, umid_max = 83, temp_min = 10, temp_max = 26, irrigacao = 0, aquecimento = 0, refrigeracao = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void register1(uint8_t value); // Protótipo das funções do registrador
void register2(uint8_t value);
void register3(uint8_t value);
void controlarSaidas(int umid, int temp);
void atualizarDisplay(int display, int dado);
int incrementoDecremento(int variavel);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
static void ADC_SET_CHANNEL(uint32_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(KIT_LED_GPIO_Port, KIT_LED_Pin, 0); // led ligado
  HAL_GPIO_WritePin(GPIOB, refrigeracaoB, 1);
  HAL_GPIO_WritePin(GPIOA, aquecimentoA, 1);
  HAL_GPIO_WritePin(GPIOA, irrigacaoA, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, refrigeracaoB, 0);
  HAL_GPIO_WritePin(GPIOA, aquecimentoA, 0);
  HAL_GPIO_WritePin(GPIOA, irrigacaoA, 0);
  register1(0);
  register2(0);
  register3(0);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    int controle = 0;
    if(!HAL_GPIO_ReadPin(GPIOB, funcao)) // Leitura do botão de função
    {
      HAL_Delay(500);
      HAL_TIM_Base_Stop_IT(&htim4);
      HAL_TIM_Base_Stop_IT(&htim5);                
      
      do 
      { // Loop de navegação do menu
        atualizarDisplay(1, menu_opcao); // Mostra a opção do menu no display 1
        switch (menu_opcao)
          {
            case 0:
              atualizarDisplay(2, 10);
              atualizarDisplay(3, 10);
              break;
            case 1: 
              atualizarDisplay(2, temp_min/10);  
              atualizarDisplay(3, temp_min%10);
              break;
            case 2:
              atualizarDisplay(2, temp_max/10);  
              atualizarDisplay(3, temp_max%10);
              break;
            case 3:
              atualizarDisplay(2, umid_min/10);  
              atualizarDisplay(3, umid_min%10);
              break;
            case 4:
              atualizarDisplay(2, umid_max/10);  
              atualizarDisplay(3, umid_max%10);
              break;
          }
        
        if (!HAL_GPIO_ReadPin(GPIOB, funcao)) 
        { // Seleciona a opção do menu
          HAL_Delay(500);
          menu_opcao++;
          if (menu_opcao > 4) menu_opcao = 0;
        }

        if (!HAL_GPIO_ReadPin(GPIOB, incremento) || !HAL_GPIO_ReadPin(GPIOB, decremento)) 
        { // Verifica se o botão de incremento ou decremento foi pressionado
          HAL_Delay(250);
          switch (menu_opcao)
          {
            case 0:
              controle = 1;
              break;
            case 1: 
              temp_min = incrementoDecremento(temp_min);
              break;
            case 2:
              temp_max = incrementoDecremento(temp_max);
              break;
            case 3:
              umid_min = incrementoDecremento(umid_min);
              break;
            case 4:
              umid_max = incrementoDecremento(umid_max);
              break;
          }
        } 
      } while (controle == 0); 
    } 
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  // Period e Prescaler configurados para 1Hz
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9600-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 9600-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KIT_LED_GPIO_Port, KIT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : KIT_LED_Pin */
  GPIO_InitStruct.Pin = KIT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KIT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA6 PA7 PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void register1(uint8_t value)
{
  HAL_GPIO_WritePin(GPIOA, shift1, 1);
  for (int i = 0; i < 8; i++) // Deslocamento dos dados
  {
    HAL_GPIO_WritePin(GPIOA, store1, GPIO_PIN_RESET); // Clock Low
    HAL_GPIO_WritePin(GPIOA, data1, (value & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Envia o bit atual
    HAL_GPIO_WritePin(GPIOA, store1, GPIO_PIN_SET); // Clock High
  }
  HAL_GPIO_WritePin(GPIOA, store1, GPIO_PIN_RESET); // Latch Low
  HAL_GPIO_WritePin(GPIOA, store1, GPIO_PIN_SET); // Latch High
  HAL_GPIO_WritePin(GPIOA, shift1, 0);
} //Fim register1

void register2(uint8_t value)
{
  HAL_GPIO_WritePin(GPIOA, shift2, 1);
  for (int i = 0; i < 8; i++) // Deslocamento dos dados
  {
    HAL_GPIO_WritePin(GPIOA, store2, GPIO_PIN_RESET); // Clock Low
    HAL_GPIO_WritePin(GPIOA, data2, (value & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Envia o bit atual
    HAL_GPIO_WritePin(GPIOA, store2, GPIO_PIN_SET); // Clock High
  }
  HAL_GPIO_WritePin(GPIOA, store2, GPIO_PIN_RESET); // Latch Low
  HAL_GPIO_WritePin(GPIOA, store2, GPIO_PIN_SET); // Latch High
  HAL_GPIO_WritePin(GPIOA, shift2, 0);
} //Fim register2

void register3(uint8_t value)
{
  HAL_GPIO_WritePin(GPIOB, shift3, 1);
  for (int i = 0; i < 8; i++) // Deslocamento dos dados
  {
    HAL_GPIO_WritePin(GPIOB, store3, GPIO_PIN_RESET); // Clock Low
    HAL_GPIO_WritePin(GPIOB, data3, (value & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // Envia o bit atual
    HAL_GPIO_WritePin(GPIOB, store3, GPIO_PIN_SET); // Clock High
  }
  HAL_GPIO_WritePin(GPIOB, store3, GPIO_PIN_RESET); // Latch Low
  HAL_GPIO_WritePin(GPIOB, store3, GPIO_PIN_SET); // Latch High
  HAL_GPIO_WritePin(GPIOB, shift3, 0);
} //Fim register3

static void ADC_SET_CHANNEL(uint32_t channel) // Função para setagem de canal (valor que ela recebe na chamada é o nome definido para a porta onde está conectado o potenciômetro que será lido)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
  if (htim->Instance == TIM4)
  {
    // Inicia a conversão do primeiro potenciômetro temperatura
    ADC_SET_CHANNEL(sensor_temp);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    analog_temp = HAL_ADC_GetValue(&hadc1);
    temp = round((float)(analog_temp*99)/4095);

    // Inicia a conversão do segundo potenciômetro umidade
    ADC_SET_CHANNEL(sensor_umid);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    analog_umid = HAL_ADC_GetValue(&hadc1);
    umid = round((float)(analog_umid*99)/4095);
    
    controlarSaidas(umid, temp);
  }

  if (htim->Instance == TIM5) {
    if (atualizacao == 1)
    {
      register1(56);
      atualizarDisplay(2, umid/10);  
      atualizarDisplay(3, umid%10);
      atualizacao = 0;
    } else {
      register1(240);
      atualizarDisplay(2, temp/10);  
      atualizarDisplay(3, temp%10); 
      atualizacao = 1;
    }    
  }
}

void controlarSaidas(int umid, int temp) {
  if (umid <= umid_min) 
  { // Umidade do solo menor ou igual a umidade mínima, ativa a saída de irrigação
    HAL_GPIO_WritePin(GPIOA, irrigacaoA, 1);
    irrigacao = 1;
  }
  else if (umid >= (umid_min + (umid_max - umid_min) * 3 / 4)) 
  { // Irrigação ativada até que se atinja a umidade mínima mais 75% do intervalo entre umidade mínima e máxima
    HAL_GPIO_WritePin(GPIOA, irrigacaoA, 0);
    irrigacao = 0;
  }

  if (temp >= temp_max) 
  { // Temperatura maior ou igual que a máxima ativa a refrigeração
    HAL_GPIO_WritePin(GPIOB, refrigeracaoB, 1);
    refrigeracao = 1;
  } 
  else if(temp < temp_max - (temp_max - temp_min) * 1 / 4) 
  { // Refrigeração ativada até que se atinja a temperatura máxima menos 25% do intervalo entre temp. mínima e máxima
    HAL_GPIO_WritePin(GPIOB, refrigeracaoB, 0);
    refrigeracao = 0;
  }
  
  if (temp <= temp_min) { // Temperatura menor ou igual que a mínima ativa o aquecimento
    HAL_GPIO_WritePin(GPIOA, aquecimentoA, 1);
    aquecimento = 1;
  } 
  else if(temp > temp_min + (temp_max - temp_min) * 1 / 4) 
  { // Aquecimento ativo até que se atinja a temperatura mínima mais 25% do intervalo entre temp. mínima e máxima
   HAL_GPIO_WritePin(GPIOA, aquecimentoA, 0);
   aquecimento = 0;
  }

  if (((temp <= temp_max - (temp_max - temp_min) * 9 / 10 && temp > temp_min) && aquecimento == 0) || 
    ((temp >= temp_min + (temp_max - temp_min) * 9 / 10 && temp < temp_max) && refrigeracao == 0) || 
    ((umid <= umid_max - (umid_max - umid_min) * 9 / 10 && umid > umid_min) && irrigacao == 0))
  {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Ativa o led de alerta para sinais dos sensores próximos aos limites
  } 
  else HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

void atualizarDisplay(int display, int dado) {
  int valores[] = {126, 12, 182, 158, 204, 218, 250, 14, 254, 222, 0};
  switch (display) {
    case 1:
      register1(valores[dado]);
      break;
    case 2:
      register2(valores[dado]);
      break;
    case 3:
      register3(valores[dado]);
      break;
  }
}

int incrementoDecremento(int variavel) {
  int flag = 0;
  while (flag == 0)
  {
    atualizarDisplay(2, variavel / 10);
    atualizarDisplay(3, variavel % 10);

    if(!HAL_GPIO_ReadPin(GPIOB, incremento))
    {
      HAL_Delay(250);
      variavel++;
    }
    if(!HAL_GPIO_ReadPin(GPIOB, decremento))
    {
      HAL_Delay(250);
      variavel--;
    }
    
    if(!HAL_GPIO_ReadPin(GPIOB, funcao)) 
    { // Verifica se o botão de função foi pressionado novamente para sair do loop
      HAL_Delay(250);
      flag = 1;
    }
  }
  return variavel;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
