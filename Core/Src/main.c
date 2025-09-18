/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
This project contains the firmware code for the STM32LK432B for the MSRL electrostimulation driver.
/*
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//if AUTOSTART is defined, the output will be automatically be enabled after startup
#define AUTOSTART
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#define ISENSTVTY 0.1 	//current sensor sensitivity in V/A
float convert_current(uint32_t adcout){
	return (3.3/4096*adcout-1.5)/(ISENSTVTY)+0.9;
}
void setplus(void){
	HAL_GPIO_WritePin(LEGAP_GPIO_Port, LEGAP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEGAN_GPIO_Port, LEGAN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEGBP_GPIO_Port, LEGBP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEGBN_GPIO_Port, LEGBN_Pin, GPIO_PIN_SET);
}
void setminus(void){
	HAL_GPIO_WritePin(LEGAP_GPIO_Port, LEGAP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEGAN_GPIO_Port, LEGAN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEGBP_GPIO_Port, LEGBP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEGBN_GPIO_Port, LEGBN_Pin, GPIO_PIN_RESET);
}
void setnull(void){
	HAL_GPIO_WritePin(LEGAP_GPIO_Port, LEGAP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEGAN_GPIO_Port, LEGAN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEGBP_GPIO_Port, LEGBP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEGBN_GPIO_Port, LEGBN_Pin, GPIO_PIN_RESET);
}
void setbothminus(void){
	HAL_GPIO_WritePin(LEGAP_GPIO_Port, LEGAP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEGAN_GPIO_Port, LEGAN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEGBP_GPIO_Port, LEGBP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEGBN_GPIO_Port, LEGBN_Pin, GPIO_PIN_SET);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UART related definitions
float duty=0.0;
uint32_t period = 1024; // initial counter corresponds to f_pwm=1.12kHz
uint8_t uart_rx_symbol;
#define UART_BUFFER_SIZE 32
uint8_t rx_buffer_counter;
uint8_t rx_buffer[UART_BUFFER_SIZE];
uint8_t tx_buffer[UART_BUFFER_SIZE];
// commands & fsm
#define COMMAND_LENGTH 4
const uint8_t CMD_START[] = "STRT";
const uint8_t CMD_STOP[] = "STOP";
const uint8_t CMD_CNTR[] = "CNTR";
// ADC related variables
#define ADC_BUFFER_SIZE 1024
uint32_t adc_val = 0;
uint16_t adc_buffer_counter = 0;
float current_meas=0.0;
float adc_buffer[ADC_BUFFER_SIZE];
// pwm frequency related variables
uint32_t des_freq_mHz = 0;

//tolerance band control relate
enum tolbandstate{
	INIT,
	VNULLTOPLUS,
	VNULLTOMINUS,
	VMINUS,
	VPLUS
};
enum tolbandstate state=INIT;
float iref=5.0;
float deltai=0.1;
bool run_main=false;
unsigned int loopcounter=0;
unsigned int counterlim=1000;

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  ADC12_COMMON->CCR |= (1U << 17U);
  HAL_Delay(1000);
  if (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK){
	  Error_Handler();
  }
  HAL_Delay(1000);
  ADC_Enable(&hadc1);
  HAL_Delay(1000);






  // setup PWM on Timer1 channel 1 and channel 2
  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period / 2);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  //htim1.Init.Period = period;
 //HAL_TIM_PWM_Init(&htim1);
  // light up LED3
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
  HAL_ADC_Start_IT(&hadc1);
  // start Timer2 which triggers ADC sampling
  HAL_TIM_Base_Start_IT(&htim2);

#ifdef AUTOSTART
  // enable half-bridge drive
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(run_main){
		  HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		//perform the state transition
		  switch(state){
		  case INIT:
			  if(current_meas<iref)
				  state=VNULLTOPLUS;
			  else
				  state=VNULLTOMINUS;
			  break;
		  case VNULLTOPLUS:
			  state=VPLUS;
			  break;
		  case VNULLTOMINUS:
			  state=VMINUS;
			  break;
		  case VMINUS:
			  if(current_meas<(iref-deltai))
				  state=VNULLTOPLUS;
			  else
				  state=VMINUS;
			  break;
		  case VPLUS:
			  if(current_meas>(iref+deltai))
				  state=VNULLTOMINUS;
			  else
				  state=VPLUS;
			  break;
		  }
		  //apply output signals
		  switch(state){
		  case INIT:
			  setbothminus();
			  break;
		  case VNULLTOPLUS:
			  setnull();
			  break;
		  case VNULLTOMINUS:
			  setnull();
			  break;
		  case VPLUS:
			  setplus();
			  break;
		  case VMINUS:
			  setminus();
			  break;
		  }

		// read a single character in interrupt mode
		HAL_UART_Receive_IT(&huart2, &uart_rx_symbol, 1);
		// HAL_UART_Transmit(&huart2,uart_tx_buffer,5,100);
		run_main=false;
		if(loopcounter>=counterlim){
			loopcounter=0;
			iref=-1*iref;
		}
		else
			loopcounter++;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 320-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_SAMPLE_CLK_Pin|LEGAN_Pin|LEGAP_Pin|LEGBP_Pin
                          |SHUTDOWN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEGBN_GPIO_Port, LEGBN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADC_SAMPLE_CLK_Pin LEGAN_Pin LEGAP_Pin LEGBP_Pin
                           SHUTDOWN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = ADC_SAMPLE_CLK_Pin|LEGAN_Pin|LEGAP_Pin|LEGBP_Pin
                          |SHUTDOWN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LEGBN_Pin */
  GPIO_InitStruct.Pin = LEGBN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEGBN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  run_main=true;
  HAL_ADC_Start_IT(&hadc1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // a command ends with the '\n' character
  if (uart_rx_symbol == '\n')
  {
    rx_buffer[rx_buffer_counter] = 0;
    process_command();
    return;
  }
  else
  {
    // write the received symbol to the buffer
    rx_buffer[rx_buffer_counter] = uart_rx_symbol;
  }
  if (rx_buffer_counter == UART_BUFFER_SIZE)
  {
    // circular buffer
    rx_buffer_counter = 0;
  }
  else
  {
    rx_buffer_counter = rx_buffer_counter + 1;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	HAL_GPIO_TogglePin(ADC_SAMPLE_CLK_GPIO_Port, ADC_SAMPLE_CLK_Pin);
  adc_buffer[adc_buffer_counter] = convert_current(HAL_ADC_GetValue(&hadc1));
  current_meas=adc_buffer[adc_buffer_counter];
  adc_buffer_counter = (adc_buffer_counter + 1) % ADC_BUFFER_SIZE;
}

void process_command(void)
{
  if (strcmp(CMD_START, rx_buffer) == 0)
  {
    // execute START command
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
  }
  if (strcmp(CMD_STOP, rx_buffer) == 0)
  {
    // execute STOP command
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);
  }
  if (strcmp(CMD_CNTR, rx_buffer) == 0)
  {
    // set the desired frequency
    uint32_t period = atoi(rx_buffer + COMMAND_LENGTH + 1);
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period / 2);
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
    //htim1.Init.Period = period;
    //HAL_TIM_PWM_Init(&htim1);
  }
  // reset the buffer counter
  rx_buffer_counter = 0;
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
