/* USER CODE BEGIN Header */

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* ---------- encoder / timing ---------- */

/* Counts and timing */
const uint32_t SAMPLE_HZ = 1; // 1 Hz (1 s)
const uint32_t SAMPLE_INTERVAL_HZ = 1000000u / SAMPLE_HZ; // 1 Hz (1 s)

volatile uint8_t encoder_print_request = 0; // запрос на отладочную печать скорости (поднимается раз в секунду по прерыванию)

volatile int16_t encoder_last_cnt = 0;     // last 16-bit reading (signed cast)
volatile int64_t encoder_total = 0;        // cumulative total ticks (64-bit to avoid overflow)
volatile int32_t encoder_delta = 0;        // last delta (ticks)
volatile float encoder_speed_tps = 0.0f;   // ticks per second (sampled)
volatile int8_t encoder_direction = 0;     // sign of speed: 1 / -1 / 0

/* encoder parameters */
const int ENCODER_TICKS_PER_MOTOR_REV = 44;   // 44 (A↑/A↓/B↑/B↓)*11 на моторном валу
const int GEAR_RATIO = 131;                  // мотор:редуктор = 131:1
const int TICKS_PER_OUTPUT_REV = ENCODER_TICKS_PER_MOTOR_REV * GEAR_RATIO;

/* periodic sampling using DWT micros()*/
static uint32_t encoder_sample_interval_us = 10000u; // 10 ms => 100 Hz
static uint32_t encoder_last_sample_us = 0u;

/* TIM clock (timer input clock Hz) */
uint32_t TIM1_clk_hz = 0u;

/* ----------motors_tests---------------------------*/
uint32_t motor_state = 0;
uint32_t motor_state_last_ts = 0;

/* ---------- DWT microsecond timer (for precise dt) ---------- */
static uint32_t dwt_cycles_per_us = 0u;
static void DWT_Init(void)
{
    /* Enable DWT cycle counter for microsecond timestamps */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* SystemCoreClock must be updated before this call */
    dwt_cycles_per_us = SystemCoreClock / 1000000u;
    if (dwt_cycles_per_us == 0u) dwt_cycles_per_us = 1u; /* guard */
}
static inline uint32_t micros(void)
{
    /* Return microseconds since DWT->CYCCNT reset */
    return (uint32_t)(DWT->CYCCNT / dwt_cycles_per_us);
}

/* ---------- ADC (current sense) ---------- */
volatile uint16_t adc_val = 0;       
volatile uint32_t adc_sample_counter = 0;

const float ADC_VREF = 3.3f;
const float ADC_MAX = 4095.0f;
const float ACS_SENS = 0.185f;     // V/A for ACS712-5A
const float VOLT_DIV_RATIO = 1.0f; // volt divider

volatile float V_zero = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
/* motor functions */
uint32_t Speed_To_Duty(int16_t speed);
void Set_Motor_Speed(int16_t speed);
void Set_Motor_Brake(int16_t speed);
static inline void set_direction_forward(void);
static inline void set_direction_backward(void);
static inline void set_direction_stop_brake(void);
static inline void set_speed(uint32_t duty);

/* encoder functions */
void encoder_init_and_start(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
float ticks_to_rad_per_sec(float ticks_per_sec);
float ticks_to_rpm(float ticks_per_sec);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM3_Init(); // PWM
  MX_ADC1_Init();
  MX_TIM2_Init(); // ADC TRGO timer
  MX_TIM1_Init(); // encoder
  MX_TIM6_Init(); // basic timer for periodic updates (speed)
  /* USER CODE BEGIN 2 */

  /* start PWM channels (motor) */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  set_direction_stop_brake();


  /* update SystemCoreClock and init DWT micros() */
  // Здесь запускаем аппаратный счётчик тактов на CPU
  SystemCoreClockUpdate();
  DWT_Init();
  //printf("DWT cycles/us = %u, micros() = %u\n", dwt_cycles_per_us, micros());


  /* compute TIM1 clock: timers on APB2 get clock = PCLK2 * (APB2_prescaler==1 ? 1 : 2) */
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq(); // returns PCLK2
  // If APB2 prescaler != 1, timer clock is doubled on STM32F4 family
  TIM1_clk_hz = ( (RCC->CFGR & RCC_CFGR_PPRE2) == RCC_CFGR_PPRE2_DIV1 ) ? pclk2 : (pclk2 * 2u);

  /* Start encoder interface (hardware encoder mode) */
  encoder_init_and_start();

  /* Configure ADC DMA (circular) and start ADC with DMA. */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 1) != HAL_OK) { Error_Handler(); }
  /* Start TIM2 (it is configured to generate TRGO for ADC) */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK) { Error_Handler(); } // TIM2 -> ADC TRGO in your config

  /* calibration: read some samples to compute V_zero */
  HAL_Delay(200);
  uint32_t sum = 0;
  const int CAL_SAMPLES = 200;
  for (int i=0; i<CAL_SAMPLES; ++i) {
      HAL_Delay(1);
      sum += (uint32_t)adc_val;
  }
  float adc_avg = (float)sum / (float)CAL_SAMPLES;
  V_zero = adc_avg * (ADC_VREF / ADC_MAX) / VOLT_DIV_RATIO;
  printf("ADC zero calibrated: adc_avg=%.1f, V_zero=%.4f V\r\n", adc_avg, V_zero);


  encoder_last_sample_us = micros();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (encoder_print_request) {
    	encoder_print_request = 0;
    	float rpm = ticks_to_rpm(encoder_speed_tps);
    	float rds = ticks_to_rad_per_sec(encoder_speed_tps);
    	printf("rpm=%.2f rds=%.2f tps=%.1f dir=%d\n", rpm, rds, encoder_speed_tps, (int)encoder_direction);
    	uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    	uint32_t tim1_clk = ( (RCC->CFGR & RCC_CFGR_PPRE2) == RCC_CFGR_PPRE2_DIV1 ) ? pclk2 : (pclk2 * 2u);
    	printf("HCLK=%lu PCLK2=%lu TIM1_clk=%lu\n", (unsigned long)SystemCoreClock, (unsigned long)pclk2, (unsigned long)tim1_clk);
    }
    
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - motor_state_last_ts;
    switch (motor_state) {
        case 0:
            Set_Motor_Speed(-800);
            motor_state_last_ts = now;
            motor_state = 1;
            break;
        case 1:
        	if (elapsed >= 3000u) {
        	    Set_Motor_Brake(0);
        	    motor_state_last_ts = now;
        	    motor_state = 2;
        	}
        	break;
        case 2:
            if (elapsed >= 3000u) {
                Set_Motor_Speed(800);
                motor_state_last_ts = now;
                motor_state = 3;
            }
            break;
        case 3:
        	if (elapsed >= 3000u) {
        	    Set_Motor_Brake(0);
        	    motor_state_last_ts = now;
        	    motor_state = 4;
        	}
        	break;
        case 4:
            if (elapsed >= 3000u) {
                motor_state_last_ts = now;
                motor_state = 0;
            }
            break;
        default:
        	motor_state = 0;
        	motor_state_last_ts = now;
        	break;
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
	  Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LPWM_Const_GPIO_Port, LPWM_Const_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RPWM_Const_GPIO_Port, RPWM_Const_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LPWM_Const_Pin */
  GPIO_InitStruct.Pin = LPWM_Const_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LPWM_Const_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RPWM_Const_Pin */
  GPIO_InitStruct.Pin = RPWM_Const_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RPWM_Const_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* PA1 -> ADC1_IN1 */

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM1_CH1 -> PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;            // alternate func
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;              // or GPIO_PULLUP depending on your hall idle level
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;         // AF1 for TIM1 on F4
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* TIM1_CH2 -> PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;              // match above
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ФУНКЦИЯ ПЕЧАТИ В КОНСОЛЬ
int _write(int file, char *ptr, int len)
{
 (void)file;
 int DataIdx;

 for (DataIdx = 0; DataIdx < len; DataIdx++)
 {
   ITM_SendChar(*ptr++);
 }
 return len;
}

uint32_t Speed_To_Duty(int16_t speed) {
    // ЗДЕСЬ ЕЩЁ В БУДУЩЕМ БУДЕМ УЧИТЫВАТЬ РАДИАНЫ В ШИМ И МЁРТВУЮ ЗОНУ
	const int16_t MAX = 999;
	if (speed > MAX) speed = MAX;
	if (speed < -MAX) speed = -MAX;

	return (uint32_t)( (speed > 0) ? speed : -speed );
}

// ФУНКЦИЯ ПЕРЕДАЧИ СКОРОСТИ НА МОТОР
void Set_Motor_Speed(int16_t speed) {
	uint32_t duty = Speed_To_Duty(speed);

	if (speed > 0) {
		/* FORWARD */
		set_direction_forward();
		set_speed(duty);
	} else if (speed < 0) {
		/* BACKWARD */  /* USER CODE BEGIN 6 */
		  /* User can add his own implementation to report the file name and line number,
		     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
		  /* USER CODE END 6 */
		set_direction_backward();
		set_speed(duty);
	}
}

// ФУНКЦИЯ ЗАПУСКАЮЩАЯ ТОРМОЖЕНИЕ
void Set_Motor_Brake(int16_t speed) {
	uint32_t duty = Speed_To_Duty(speed);
	set_direction_stop_brake();
	set_speed(duty);
}

/* ФУНКЦИИ НИЗКОГО УРОВНЯ*/
static inline void set_direction_forward(void) {
	/* L_PWM = LOW, R_PWM = HIGH*/
	HAL_GPIO_WritePin(LPWM_Const_GPIO_Port, LPWM_Const_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RPWM_Const_GPIO_Port, RPWM_Const_Pin, GPIO_PIN_SET);
}

static inline void set_direction_backward(void) {
	/* L_PWM = HIGH, R_PWM = LOW*/
	HAL_GPIO_WritePin(LPWM_Const_GPIO_Port, LPWM_Const_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RPWM_Const_GPIO_Port, RPWM_Const_Pin, GPIO_PIN_RESET);
}

static inline void set_direction_stop_brake(void) {
	/* L_PWM = HIGH, R_PWM = HIGH*/
	HAL_GPIO_WritePin(LPWM_Const_GPIO_Port, LPWM_Const_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RPWM_Const_GPIO_Port, RPWM_Const_Pin, GPIO_PIN_SET);
}

static inline void set_speed(uint32_t duty) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty); // PC9
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty); // PC8
}


/* ---------- ADC conv complete callback (DMA writes adc_val) ---------- */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    (void)hadc;
    adc_sample_counter++;

    float V_adc = ((float)adc_val) * (ADC_VREF / ADC_MAX);
    float V_sensor = V_adc / VOLT_DIV_RATIO;
    float I_meas = (V_sensor - V_zero) / ACS_SENS;

    const uint32_t PRINT_EVERY = 100;
    static uint32_t local_cnt = 0;
    if (++local_cnt >= PRINT_EVERY) {
        local_cnt = 0;
        printf("ADC=%u, V_adc=%.4f V, V_sensor=%.4f V, I=%.3f A\r\n",
               (unsigned)adc_val, V_adc, V_sensor, I_meas);
    }
}

/* ---------------- encoder helpers ---------------- */

void encoder_init_and_start(void)
{
    /* Read initial counter and set last */
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    //printf("TIM1 started, counter=%u\n", (unsigned)cnt);
    encoder_last_cnt = (int16_t)cnt;
    encoder_total = 0;
    encoder_delta = 0;
    encoder_speed_tps = 0.0f;
    encoder_direction = 0;

    /* Start TIM1 in encoder interface mode (both channels) */
    if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK) { Error_Handler(); }
}

/* вызов поднятия флага печати по прерыванию базового таймера 6 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
    	uint16_t cnt16 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    	int16_t cnt_s = (int16_t)cnt16;                 /* signed view for overflow handling */
    	int16_t last_s = encoder_last_cnt;
    	int32_t diff = (int32_t)(cnt_s - last_s);       /* signed subtraction handles wrap-around */
    	encoder_last_cnt = cnt_s;
    	encoder_delta = diff;
    	encoder_total += diff;
    	// вычисляем tps, опираясь на частоту опроса
    	encoder_speed_tps = (float)diff * (float)SAMPLE_HZ; // умножить на частоту всё равно что поделить на период
    	encoder_direction = (encoder_speed_tps) ? 1 : (encoder_speed_tps < 0 ? -1 : 0); // хотя скорость и так понятна из знака при _tps можем её явно вычислить
    	// поднимаем флаг печати
    	encoder_print_request = 1;
    }
}

/* conversion helpers */
float ticks_to_rad_per_sec(float ticks_per_sec)
{
    /* rad/s = ticks/s * (2*pi / ticks_per_rev) */
    return ticks_per_sec * (2.0f * 3.14159265358979323846f / (float)TICKS_PER_OUTPUT_REV);
}

float ticks_to_rpm(float ticks_per_sec)
{
    /* rpm = (ticks/sec) / ticks_per_rev * 60 */
    return (ticks_per_sec / (float)TICKS_PER_OUTPUT_REV) * 60.0f;
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
