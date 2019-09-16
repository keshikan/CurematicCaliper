/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ssd1306.h"
#include "ugui.h"
#include "dfilter.h"
#include "caliper.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_LENGTH (128)
#define FILT_WAIT_CYCLE (8000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

//for uGUI
UG_GUI gui;

//for ADC
uint16_t sens_val[ADC_LENGTH];
uint16_t y[2]={0,0};

uint16_t prev_data[2]={0,0};

//for DAC
uint16_t out_val[2];

//for switch
SwitchState sw_state[SW_NUM];

//for CDC

char str[24];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void pset(UG_S16 x, UG_S16 y, UG_COLOR col)
{
	displayDrawDot(x, y, col);
}

//call from main loop
void swDetect()
{
	#define SW_CNT_THR (5)		//n-time read. To avoid contact bounce.

	GPIO_TypeDef* sw_gpio_arr[SW_NUM] = {GPIOA, GPIOA};
	uint16_t sw_pin_arr[SW_NUM] = {GPIO_PIN_3, GPIO_PIN_4};
	static GPIO_PinState sw_prev_state[SW_NUM] = {GPIO_PIN_SET, GPIO_PIN_SET};
	GPIO_PinState sw_buf;
	static uint8_t sw_counter[SW_NUM] = {0, 0};

	for(uint32_t i=0; i<SW_NUM; i++){
		sw_buf = HAL_GPIO_ReadPin(sw_gpio_arr[i], sw_pin_arr[i]);
		if(sw_prev_state[i] == sw_buf){
			sw_counter[i]++;
		}else{
			sw_counter[i] = 0;
		}
		sw_prev_state[i] = sw_buf;

		// inactivate edge
		if(SW_PUSH_EDGE == sw_state[i]){
			sw_state[i] = SW_PUSH_CONTINUE;
		}else if(SW_UNPUSH_EDGE == sw_state[i]){
			sw_state[i] = SW_UNPUSH_CONTINUE;
		}

		// generate edge
		if(sw_counter[i] >= SW_CNT_THR){
			sw_counter[i] = 0;
			if(GPIO_PIN_RESET == sw_buf){

				switch (sw_state[i]) {
					case SW_PUSH_CONTINUE:
						sw_state[i] = SW_PUSH_CONTINUE;
						break;
					case SW_UNPUSH_CONTINUE:
						sw_state[i] = SW_PUSH_EDGE;
						break;
					default:
						break;
				}

			}else{

				switch (sw_state[i]) {
					case SW_PUSH_CONTINUE:
						sw_state[i] = SW_UNPUSH_EDGE;
						break;
					case SW_UNPUSH_CONTINUE:
						sw_state[i] = SW_UNPUSH_CONTINUE;
						break;
					default:
						break;
				}

			}
		}
	}

}

//reference https://umtkm.github.io/2017/12/19/nucleo-f401-flash-struct/
void writeFlash(uint32_t address, uint64_t *data, uint32_t size)
{
	HAL_StatusTypeDef hstat;

	HAL_FLASH_Unlock();

	//erase
	uint32_t pageError = 0;
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Page = 63;
	erase.NbPages = 1;
	erase.Banks = FLASH_BANK_1;
	hstat = HAL_FLASHEx_Erase(&erase, &pageError);

	if (HAL_OK != hstat )
	{
		HAL_FLASH_Lock();
		return;
	}

	if(0xFFFFFFFF != pageError){
		HAL_FLASH_Lock();
		return;
	}

	//write
	for(uint32_t i=0; i<size; i++){
		hstat = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *data );
		address += 8;
		data++;

		if (HAL_OK != hstat )
		{
			HAL_FLASH_Lock();
			return;
		}
	}

	HAL_FLASH_Lock();
}

void loadFlash(uint32_t address, uint8_t *data, uint32_t bytesize)
{
	memcpy(data, (uint8_t*)address, bytesize);
}

bool verifyFlash()
{
	CalibrationData cd;

	loadFlash(CALIB_DATA_ADDRESS, (uint8_t*)&cd, sizeof(calib_dat));

	  if(CALIB_DATA_CHECK_NUM == cd.chk_num){
		  return true;
	  }

	  return false;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t finish_cnt=0;
	CalibStatus cstat;

	if(htim->Instance == TIM6)
	{
		//sensing
		for(uint32_t i=0; i<2; i++){
			y[i] =  dfRCFilt(sens_val[i], prev_data[i]);
			prev_data[i] =y[i];
		}

		  //mode selection
		  switch(cpGetCaliperMode()){

			  case SCALE_CALIB:
				  cstat = cpCalibration(y);
				  if(CALIB_FINISHED == cstat){
					  cpSetCaliperMode(SCALE_CALIB_FINISH);
					  finish_cnt=0;
				  }else if(CALIB_NG == cstat){
					  cpCalibrationStart();
				  }
				  break;

			  case SCALE_CALIB_FINISH:
				  if(100 <= finish_cnt){
					  cpSetCaliperMode(SCALE_WORK);
				  }else{
					  finish_cnt++;
				  }
				  break;

			  case SCALE_WORK:

				  cpApplyPhaseCompensation(y);
				  cpCalcLength(y[0],y[1]);
				  break;

			  case SCALE_HOLD:
				  break;
			  default:
				  break;
		  }

	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){

	if(hi2c->Instance == I2C1)
	{
		disp_transmitted = true;
		HAL_I2C_Master_Transmit_IT(&hi2c1, 0x78, (uint8_t *)disp_memory, DISP_MEMORY_SIZE*2);
		disp_transmitted = false;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static ScaleMode smode_prev = SCALE_CALIB;
	char send_str[20];

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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  displayInit();

   //Init ugui
  UG_Init(&gui, pset, VRAM_X_SIZE, VRAM_Y_SIZE);
  UG_SetBackcolor(C_BLACK);
  UG_SetForecolor(C_WHITE);
  UG_FillScreen(C_BLACK);

  //Init caliper
  cpInit();

  //load and check calib data
  loadFlash(CALIB_DATA_ADDRESS, (uint8_t*)&calib_dat, sizeof(calib_dat));
  if(CALIB_DATA_CHECK_NUM == calib_dat.chk_num){
		for(uint32_t i=0; i<SENSOR_NUM; i++){
			c_state.amplitude[i] = calib_dat.amplitude[i] ;
			c_state.max[i] = calib_dat.max[i];
			c_state.middle[i] = calib_dat.middle[i];
			c_state.min[i] = calib_dat.min[i];
		}
		c_state.delta = calib_dat.delta;
  }else{//need calibration
		c_state.smode = SCALE_CALIB;
  }

  //ADC start
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)sens_val, ADC_LENGTH);

  //wait for stabilization of filter.
  for(uint32_t j=0; j<FILT_WAIT_CYCLE; j++){
	  for(uint32_t i=0; i<2; i++){
		  y[i] = prev_data[i] = dfRCFilt(sens_val[i], prev_data[i]);
	  }
	  cpCalcLength(y[0],y[1]);
  }
  cpSetZeroPosition();

  //display timer start
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  switch(cpGetCaliperMode()){

		  case SCALE_CALIB:
			  if(smode_prev != SCALE_CALIB){
				  smode_prev = SCALE_CALIB;
				  UG_FillScreen(C_BLACK);
				  UG_FontSelect(&FONT_8X8);
			  }
			  UG_FontSelect(&FONT_10X16);
			  sprintf(str, "CALIB MODE");
			  UG_PutString(5, 0, str);
			  UG_FontSelect(&FONT_8X8);
			  sprintf(str, "<= MOVE =>");
			  UG_PutString(15, 23, str);

			  break;

		  case SCALE_CALIB_FINISH:
			  if(smode_prev != SCALE_CALIB_FINISH){
				  smode_prev = SCALE_CALIB_FINISH;

				  /////save calibration data
				  //erase calib data
				  writeFlash(CALIB_DATA_ADDRESS, (uint64_t*)&calib_dat, (sizeof(calib_dat)/8) + ( (sizeof(calib_dat)%8) >= 1) );

				  //verify
				  verifyFlash();

				  //displahy
				  UG_FillScreen(C_BLACK);
				  UG_FontSelect(&FONT_10X16);
				  sprintf(str, "CALIB OK!");
				  UG_PutString(5, 0, str);
				  HAL_Delay(1500);
			  }

			  break;

		  case SCALE_WORK:
			  if(smode_prev != SCALE_WORK){
				  smode_prev = SCALE_WORK;
				  UG_FillScreen(C_BLACK);
				  UG_FontSelect(&FONT_12X20);

				  cpSetZeroPosition();
			  }

#ifdef CFG_DEBUG_INDICATE_ON
		  		UG_FontSelect(&FONT_4X6);
				sprintf(str, "M:%04d,%04d,%04d", y[0],c_state.max[0],c_state.min[0]);
				UG_PutString(0, 0, str);

				sprintf(str, "S:%04d,%04d,%04d", y[1],c_state.max[1],c_state.min[1]);
				UG_PutString(0, 10, str);
				sprintf(str, "D:%f",c_state.delta);
				UG_PutString(0, 10, str);

				sprintf(str, "%5.2fmm + %5.2fmm", cpGetLength(), cpGetAccLength());
				UG_PutString(0, 20, str);
#else
		  		sprintf(str, "%7.2fmm", cpGetAccLength());
		  		UG_PutString(0, 7, str);
#endif
		  	  if(SW_PUSH_EDGE == sw_state[0]){
		  		  cpSetZeroPosition();
		  	  }
			  break;

		  case SCALE_HOLD:
			  if(smode_prev != SCALE_HOLD){
				  smode_prev = SCALE_HOLD;
			  }
			  break;
		  default:
			  break;
	  }

	  swDetect();
  	  if(SW_PUSH_EDGE == sw_state[1]){
  		  cpCalibrationStart();
  	  }

  	sprintf(send_str, "LEN=%6.2f\n\r", cpGetAccLength());

    CDC_Transmit_FS((uint8_t *)send_str, sizeof(send_str));

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the SYSCFG APB clock 
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /** Configures CRS 
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
  hi2c1.Init.Timing = 0x006D215F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.Prescaler = 299;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 20;
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
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
