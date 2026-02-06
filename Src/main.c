/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

#include "mc_type.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Active/desactive partie de code
#define ON_OFF 0
#define Vitesse 0
#define cmd_rasb 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch4_trig_com;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile uint8_t fault_latched = 0;

//Ajoute un ID d'ack
uint8_t ID_order_ack = 0x40; //0x40 = ACK FAULT

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//DEF ID (trame UART) TX
uint8_t ID_vitesse = 0x00;
uint8_t ID_current = 0x01;
uint8_t ID_voltage = 0x02;
uint8_t ID_position = 0x03;
uint8_t ID_message = 0x10;
uint8_t ID_error_temp = 0x20;

//DEF ID (trame UART) RX
uint8_t ID_order_start = 0x10;
uint8_t ID_order_stop = 0x20;
uint8_t ID_order_reset = 0x30;
uint8_t ID_order_vitesse = 0x00;

//Variable measure
int16_t measure_vitesse = 0;
int16_t measure_current = 0;
int16_t measure_voltage = 0;
int16_t measure_temp = 0;
int16_t measure_pos = 0;

//Variable ordre
uint16_t vitesse = 65;
uint16_t value_order = 0;
//COM UART
volatile uint8_t buffer_RX[5];
uint8_t nbr_octet_tram = 3;

//Error
char buffer_error[64];
uint16_t faults=0;

//Commande boucle ouverte
 float w = 2*3.14;
 float angle_elec = 0;
 int16_t v_alpha = 0;//Debug
 int16_t v_beta = 0;//Debug


// Flags
volatile uint8_t new_order = 0;

 struct _AppFlags_t {
   uint8_t t1;
   uint8_t read_sensors;
 } AppFlags = {0};

// Timers
 struct _AppTimer_t {
   unsigned int delayInMs;
   uint8_t* flagPtr;
 } AppTimers[] = {
   // Intervalle (ms)  Flag
   { 100,              &AppFlags.t1 },
   { 100,             &AppFlags.read_sensors }
 };

// Fonctions
 void transmission_message(const char *message, uint8_t ID_message)
 {
	 HAL_UART_Transmit(&huart1, &ID_message, 1, 100);
	 HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 100);
 }

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_MotorControl_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, buffer_RX, nbr_octet_tram);//start RX
    //PREAMBULE
    HAL_GPIO_TogglePin (LED_VERTE_GPIO_Port, LED_VERTE_Pin);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin (LED_VERTE_GPIO_Port, LED_VERTE_Pin);
    transmission_message("COM UART OK!\n", ID_message);
    HAL_Delay(1000);
    //FIN PREAMBULE

    //Commande courant
    qd_t Iqd;
    Iqd.q = 1;
    Iqd.d = 0;

    //Commande boucle ouverte
    alphabeta_t Valpha_beta;
    Valpha_beta.alpha = 0;
    Valpha_beta.beta = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Génération du temps */
    uint32_t tickstart = HAL_GetTick();
    static uint32_t lastTick = 0;

    /* 100 ms = 200 ticks car 0.5 ms par tick */
    if ((tickstart - lastTick) >= 200)
    {
      lastTick += 200;
      AppFlags.read_sensors = 1;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ON_OFF
	  MC_StartMotor1();
	  HAL_Delay(10000);
	  MC_StopMotor1();
	  HAL_Delay(10000);
#else
#if Vitesse
	  MC_ProgramSpeedRampMotor1(vitesse, 5000);
	  MC_StartMotor1();
	  if(vitesse == 10)
	  {
		  vitesse = 1000;
	  }
	  else
	  {
		  vitesse = 10;
	  }
	  HAL_Delay(5000);
	  MC_StopMotor1();
	  HAL_Delay(5000);
#else
#if cmd_rasb
	  // MC_GetMecSpeedAverageMotor1();//MC_GetImposedDirectionMotor1();
	  // mc_interface.h ==> CODE des mots clés (IDLE,START....)
	  // mc_type.h ==> CODE des mots clés erreur
	  // mc_stm_types.h  ==> SPEED_UNIT et U_RPM
	  // parametre par defaut ==> parameters_conv (vitesse ligne 75)
	  // pwm_curr_fdbk.h ==> #define SQRT3FACTOR ((uint16_t)0xDDB4) /* = (16384 * 1.732051 * 2)*/
	  // Attention le tableau réel est msg2[0] = R, msg2[1] = e ... , msg2[9] = T, msg2[9] = \n, msg2[10] = \0 (0x00)
	  // Temperature (ntc_temperature_sensor.c in Middleware/Motorcontrol/) :
MCI_State_t state = MC_GetSTMStateMotor1();
static MCI_State_t last_state = IDLE;

/* N'envoie l'état qu'à chaque transition, pas en boucle */
if(state != last_state)
{
  last_state = state;
  sprintf(buffer_error, "Transition d'état : %u\n", (unsigned)state);
  transmission_message(buffer_error, ID_message);
}

/* Mettre à jour la température */
measure_temp = NTC_GetAvTemp_C(&TempSensor_M1); 

/* 0 - Protection température */
if(measure_temp >= 100)
{
  MC_StopMotor1();
  HAL_UART_Transmit(&huart1, &ID_error_temp, 1, 1000);
  continue; //ne pas exécuter le reste du code
}

/*1 - Détection FAULT -> latch + stop*/
if ((state == FAULT_NOW) || (state == FAULT_OVER))
{
  if (!fault_latched) //ne pas spammer en boucle
  {
    fault_latched = 1;

    //Stop moteur
     MC_StopMotor1();

    //Log UART
    transmission_message("ERREUR\n", ID_message);
    sprintf(buffer_error, "Etat = %u\n", state);
    transmission_message(buffer_error,ID_message);

    faults = MC_GetOccurredFaultsMotor1();
    sprintf(buffer_error, "Erreur : %04X\n", faults);
    transmission_message(buffer_error, ID_message);

    transmission_message("Attente ACK ou RESET\n", ID_message);
  }
}

/*2 - Si fault latched : on bloque toute action moteur (sauf ACK/RESET)*/
if (fault_latched)
{
  //Renvoie l'état périodiquement
  //On ne fait pas la lecture capteurs / ordre moteur normalement
  //On traite uniquement les ordres UART liés à la sécurité 
  if(new_order)
  {
    new_order = 0;

    if(buffer_RX[0] == ID_order_stop)
    {
      MC_StopMotor1();
      transmission_message("STOP OK (fault latched)\n", ID_message);
    }
    else if((buffer_RX[0] == ID_order_reset) || (buffer_RX[0] == ID_order_ack))
    {
      MC_StopMotor1();

      if(buffer_RX[0] == ID_order_reset)
      {
        vitesse = 500;
        transmission_message("RESET demande\n", ID_message);
      }
      else
      {
        transmission_message("ACK FAULT demande\n", ID_message);
      }

      MC_AcknowledgeFaultMotor1(); //tentative de réarmement
    
      //On relit l'état pour savoir si le fault est parti
      state =  MC_GetSTMStateMotor1();
      if((state != FAULT_NOW) && (state != FAULT_OVER))
      {
        fault_latched = 0;
        transmission_message("Fault Cleared (IDLE)\n", ID_message);
      }
      else
      {
        transmission_message("Fault encore present\n", ID_message);
      }
    }
    else
    {
      transmission_message("Ignore Fault : Latched\n", ID_message);
    }
  }

  //Sort ici pour éviter de continuer le code normal
  //Dans un while(1), ça continue la prochaine itération
  continue;
}

/*3 - Pas de fault latched : fonctionnement normal*/
	if(AppFlags.read_sensors)
		{
	    AppFlags.read_sensors = 0;

	  measure_vitesse = MC_GetMecSpeedAverageMotor1();
		HAL_UART_Transmit(&huart1, &ID_vitesse, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t*)&measure_vitesse, sizeof(measure_vitesse), 1000);

		measure_current = MC_GetPhaseCurrentAmplitudeMotor1();
		HAL_UART_Transmit(&huart1, &ID_current, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t*)&measure_current, sizeof(measure_current), 1000);

		measure_voltage = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1);
		HAL_UART_Transmit(&huart1, &ID_voltage, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t*)&measure_voltage, sizeof(measure_voltage), 1000);

		measure_temp = NTC_GetAvTemp_C(&TempSensor_M1);

		measure_pos = (int16_t)(MC_GetCurrentPosition1()*100);
		HAL_UART_Transmit(&huart1, &ID_position, 1, 1000);
		HAL_UART_Transmit(&huart1, (uint8_t*)&measure_pos, sizeof(measure_pos), 1000);

		if(state == RUN)
			{
			//MC_SetCurrentReferenceMotor1(Iqd); //Commande en courant
			MC_ProgramSpeedRampMotor1(vitesse, 1000); //Commande en vitesse
			transmission_message("RUN\n", ID_message);

			//Commande en boucle ouverte (Test)
			//Valpha_beta.alpha = (int16_t)(50*cos(angle_elec)*10);
			//Valpha_beta.beta  = (int16_t)(50*sin(angle_elec)*10);
			//	v_beta = Valpha_beta.beta; //Debug
			//  v_alpha = Valpha_beta.alpha;//Debug
			//PWMC_SetPhaseVoltage(pwmcHandle[M1], Valpha_beta);
			//angle_elec += 0.1;
			//angle_elec += w*0.05; //Tour mécanique en 8s
			/*if(angle_elec >= (2*3.14))
				{
				angle_elec =  0;
				}*/
			//Fin commande en boucle ouverte (Test)

			}
		}
	if(new_order)
		{
		new_order = 0;
		if(buffer_RX[0] == ID_order_vitesse)//Ordre de vitesse
			{
			value_order = ((buffer_RX[2] << 8) | buffer_RX[1]);
			if((value_order < 50))
				vitesse = 50;
			else if(value_order >500)
				vitesse = 500;
			else
				vitesse = value_order;
			}
		else if(buffer_RX[0] == ID_order_start)//Ordre de start
			{
			MC_ProgramSpeedRampMotor1(vitesse, 1000);
			MC_StartMotor1();
			}
		else if(buffer_RX[0]== ID_order_stop)//Ordre de stop
			{
			MC_StopMotor1();
			}
		else if (buffer_RX[0] == ID_order_reset)//Ordre de reset
			{
			MC_StopMotor1();
			vitesse = 500;
			}
		}
	}
#else

#endif
#endif
#endif
  }
  /* USER CODE END 3 */


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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* ADC1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO2;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.QueueInjectedContext = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC5REF_RISING_OC6REF_RISING;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
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
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) + 1);
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_6) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_LOW;
  sBreakDeadTimeConfig.Break2Filter = 3;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = M1_PULSE_NBR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_VERTE_GPIO_Port, LED_VERTE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Start_Stop_Pin */
  GPIO_InitStruct.Pin = Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_ENCODER_Z_Pin */
  GPIO_InitStruct.Pin = M1_ENCODER_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M1_ENCODER_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_VERTE_Pin */
  GPIO_InitStruct.Pin = LED_VERTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_VERTE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  new_order = 1;
  HAL_UART_Receive_IT(&huart1, buffer_RX, nbr_octet_tram);
}
/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
 	 HAL_GPIO_WritePin(LED_VERTE_GPIO_Port,LED_VERTE_Pin , GPIO_PIN_SET);
}*/
//	Fonction pour les timers
void HAL_SYSTICK_Callback(void)
{
  //for (unsigned int appTimerIndex = 0;
    //appTimerIndex < sizeof(AppTimers) / sizeof(struct _AppTimer_t);
    //appTimerIndex++)
  //{
    //if (HAL_GetTick() % AppTimers[appTimerIndex].delayInMs == 0)
    //{
        //*AppTimers[appTimerIndex].flagPtr = 1;
    //}
  //}
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
#ifdef USE_FULL_ASSERT
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

