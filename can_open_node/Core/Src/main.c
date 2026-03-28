/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include "301/CO_ODinterface.h"
#include "301/CO_SDOclient.h"
#include "OD.h"
#include "CO_app_STM32.h"
#include "CANopen.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define CAN_BUFFER_LEN          (8)
#define CONTROLLER_NODE_ID 0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
MDF_HandleTypeDef AdfHandle0;
MDF_FilterConfigTypeDef AdfFilterConfig0;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

extern CO_t *CO;

static FDCAN_FilterTypeDef   sFilterConfig;

uint8_t gSdoData[CAN_BUFFER_LEN];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADF1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CO_SDO_abortCode_t
read_SDO(CO_SDOclient_t* SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t* buf, size_t bufSize,
         size_t* readSize) {
    CO_SDO_return_t SDO_ret;

    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // upload data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }

        //sleep_us(timeDifference_us);
        HAL_Delay(timeDifference_us/1000);
    } while (SDO_ret > 0);

    // copy data to the user buffer (for long data function must be called several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

    return CO_SDO_AB_NONE;
}


CO_SDO_abortCode_t
write_SDO(CO_SDOclient_t* SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t* data, size_t dataSize) {
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;

    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return -1;
    }

    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return -1;
    }

    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
    if (nWritten < dataSize) {
        bufferPartial = true;
        // If SDO Fifo buffer is too small, data can be refilled in the loop.
    }

    // download data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientDownload(SDO_C, timeDifference_us, false, bufferPartial, &abortCode, NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }

        //sleep_us(timeDifference_us);
        HAL_Delay(timeDifference_us/1000);
    } while (SDO_ret > 0);

    return CO_SDO_AB_NONE;
}


int __io_putchar(int ch)
{
    /* Support printf over UART */
    (void)HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFFU);
    return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == canopenNodeSTM32->timerHandle) {
        canopen_app_interrupt();
    }
}

void my_fdcan1_transmit()
{
	FDCAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[] = {'C', 'A', 'N', 'I','S','F','U','N'}; // Example data

	// 1. Configure the TX Header
	TxHeader.Identifier = 0x111U;                // CAN ID
	TxHeader.IdType = FDCAN_STANDARD_ID; //standard or extended
	TxHeader.TxFrameType = FDCAN_DATA_FRAME; //data frame or request frame
	//TODO: Data length
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;    // Length 4 bytes
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;     // No BRS for classic CAN
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;      // Classic or FD format
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	// 2. Transmit the message
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
	{
		printf("Stm32u585 Error calling HAL_FDCAN_AddMessageToTxFifoQ\r\n");
	  // Transmission error
	  Error_Handler();
	}
	// No need to poll, tx complete callback will be called in case of interrupts
//	while(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != 0);
//	printf("Stm32u585 Transmitted %lu bytes\r\n",TxHeader.DataLength);

}

void my_fdcan1_receive()
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[16];

	/* check if FIFO 0 receive at least one message */
	while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) < 1U);

	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		printf("Stm32u585 Error calling HAL_FDCAN_GetRxMessage\r\n");
		Error_Handler();
	}
	else
	{
		printf("Message Received HAL_FDCAN_GetRxMessage\r\n");
		for(int i = 0; i < 8; ++i)
		{
			printf("RxData[%d] = %c\r\n",i,RxData[i]);
		}
	}
}


void my_config_filter()
{
	/* Configure Rx filter */
	sFilterConfig.IdType       = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex  = 0U;
	sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1    = 0x111U;
	sFilterConfig.FilterID2    = 0x7FFU;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		printf("Stm32u585 Error calling HAL_FDCAN_ConfigFilter\r\n");
		Error_Handler();
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	printf("Stm32u585 User Button pressed\r\n");
	my_fdcan1_transmit();
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
}

//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//	printf("Stm32u585 Transmitted HAL_FDCAN_RxFifo0Callback\r\n");
//	FDCAN_RxHeaderTypeDef RxHeader;
//	uint8_t RxData[16];
//
//	/* No need to poll */
//	//while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) < 1U);
//
//	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//	{
//		printf("Stm32u585 Error calling HAL_FDCAN_GetRxMessage in ISR\r\n");
//		Error_Handler();
//	}
//	else
//	{
//		printf("Message Received HAL_FDCAN_GetRxMessage in ISR\r\n");
//		printf("Message:");
//		for(int i = 0; i < 8; ++i)
//		{
//			printf("%c",RxData[i]);
//		}
//		printf("\r\n");
//	}
//}

//void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
//{
//	printf("Stm32u585 Transmitted HAL_FDCAN_RxFifo1Callback\r\n");
//}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
}

//void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
//{
//	//printf("Stm32u585 Transmitted HAL_FDCAN_TxBufferCompleteCallback\r\n");
//}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADF1_Init();
  MX_FDCAN1_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Configure Config filter for Rx*/
  // my_config_filter();

  /* Activate FDCAN interrupts */
//  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* Start the FDCANx module */
//  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
//  {
//	  printf("Stm32u585 Error calling HAL_FDCAN_Start\r\n");
//	  Error_Handler();
//  }
  printf("Stm32u585 testing\r\n");
  CANopenNodeSTM32 canOpenNodeSTM32;
  canOpenNodeSTM32.CANHandle = &hfdcan1;
  canOpenNodeSTM32.HWInitFunction = MX_FDCAN1_Init;
  canOpenNodeSTM32.timerHandle = &htim2;
  canOpenNodeSTM32.desiredNodeID = CONTROLLER_NODE_ID;
  canOpenNodeSTM32.baudrate = 1000;  // in kb/s
  canopen_app_init(&canOpenNodeSTM32); // CO initialised in here!
  // my_fdcan1_transmit();

  // my_fdcan1_receive();
  uint8_t tmp = 0x20;
  size_t readBytes;
  write_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, 0x10, 0x6000, 0, &tmp, 1 );
  read_SDO(canOpenNodeSTM32.canOpenStack->SDOclient, 0x10, 0x1000, 0, &gSdoData[0], 4, &readBytes );

  while (1)
  {
	  HAL_GPIO_WritePin(GPIOH, LED_RED_Pin, !canOpenNodeSTM32.outStatusLEDRed);
	  HAL_GPIO_WritePin(GPIOH, LED_GREEN_Pin, !canOpenNodeSTM32.outStatusLEDGreen);
	  canopen_app_process();

	  OD_PERSIST_COMM.x6000_velocity++;
	  //printf("Stm32u585 testing canOpenNodeSTM32.activeNodeID = %d\r\n",canOpenNodeSTM32.activeNodeID);
	  //OD_set_u32(OD_find(OD, 0x6001),0x00, 543, false);
	  //CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[0]);
	  HAL_Delay(10);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ADF1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADF1_Init(void)
{

  /* USER CODE BEGIN ADF1_Init 0 */

  /* USER CODE END ADF1_Init 0 */

  /* USER CODE BEGIN ADF1_Init 1 */

  /* USER CODE END ADF1_Init 1 */

  /**
    AdfHandle0 structure initialization and HAL_MDF_Init function call
  */
  AdfHandle0.Instance = ADF1_Filter0;
  AdfHandle0.Init.CommonParam.ProcClockDivider = 1;
  AdfHandle0.Init.CommonParam.OutputClock.Activation = DISABLE;
  AdfHandle0.Init.SerialInterface.Activation = ENABLE;
  AdfHandle0.Init.SerialInterface.Mode = MDF_SITF_LF_MASTER_SPI_MODE;
  AdfHandle0.Init.SerialInterface.ClockSource = MDF_SITF_CCK0_SOURCE;
  AdfHandle0.Init.SerialInterface.Threshold = 4;
  AdfHandle0.Init.FilterBistream = MDF_BITSTREAM0_FALLING;
  if (HAL_MDF_Init(&AdfHandle0) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    AdfFilterConfig0 structure initialization

    WARNING : only structure is filled, no specific init function call for filter
  */
  AdfFilterConfig0.DataSource = MDF_DATA_SOURCE_BSMX;
  AdfFilterConfig0.Delay = 0;
  AdfFilterConfig0.CicMode = MDF_ONE_FILTER_SINC4;
  AdfFilterConfig0.DecimationRatio = 2;
  AdfFilterConfig0.Gain = 0;
  AdfFilterConfig0.ReshapeFilter.Activation = DISABLE;
  AdfFilterConfig0.HighPassFilter.Activation = DISABLE;
  AdfFilterConfig0.SoundActivity.Activation = DISABLE;
  AdfFilterConfig0.AcquisitionMode = MDF_MODE_ASYNC_CONT;
  AdfFilterConfig0.FifoThreshold = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  AdfFilterConfig0.DiscardSamples = 0;
  /* USER CODE BEGIN ADF1_Init 2 */

  /* USER CODE END ADF1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 4;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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
  htim2.Init.Prescaler = 15999;
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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UCPD_PWR_GPIO_Port, UCPD_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WRLS_WKUP_B_GPIO_Port, WRLS_WKUP_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Mems_STSAFE_RESET_Pin|WRLS_WKUP_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WRLS_FLOW_Pin Mems_VLX_GPIO_Pin Mems_INT_LPS22HH_Pin */
  GPIO_InitStruct.Pin = WRLS_FLOW_Pin|Mems_VLX_GPIO_Pin|Mems_INT_LPS22HH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : WRLS_UART4_RX_Pin WRLS_UART4_TX_Pin */
  GPIO_InitStruct.Pin = WRLS_UART4_RX_Pin|WRLS_UART4_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_UCPD_CC1_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_CC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_UCPD_CC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_F_NCS_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_F_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(OCTOSPI_F_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_IO5_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_IO5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTOSPI_F_IO7_Pin OCTOSPI_F_IO5_Pin OCTOSPI_F_IO6_Pin OCTOSPI_F_IO4_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_F_IO7_Pin|OCTOSPI_F_IO5_Pin|OCTOSPI_F_IO6_Pin|OCTOSPI_F_IO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3_BOOT0_Pin */
  GPIO_InitStruct.Pin = PH3_BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PH3_BOOT0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_PWR_Pin */
  GPIO_InitStruct.Pin = UCPD_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WRLS_SPI2_MOSI_Pin WRLS_SPI2_MISO_Pin WRLS_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = WRLS_SPI2_MOSI_Pin|WRLS_SPI2_MISO_Pin|WRLS_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_DQS_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_DQS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_DQS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_IO7_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_IO7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTOSPI_F_IO0_Pin OCTOSPI_F_IO1_Pin OCTOSPI_F_IO2_Pin OCTOSPI_F_IO3_Pin
                           OCTOSPI_F_CLK_P_Pin OCTOSPI_F_DQS_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_F_IO0_Pin|OCTOSPI_F_IO1_Pin|OCTOSPI_F_IO2_Pin|OCTOSPI_F_IO3_Pin
                          |OCTOSPI_F_CLK_P_Pin|OCTOSPI_F_DQS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH4 PH5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin Mems_VL53_xshut_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTOSPI_R_IO0_Pin OCTOSPI_R_IO2_Pin OCTOSPI_R_IO1_Pin OCTOSPI_R_IO3_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO0_Pin|OCTOSPI_R_IO2_Pin|OCTOSPI_R_IO1_Pin|OCTOSPI_R_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_IO4_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_IO4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_C_P_Pin USB_C_PA11_Pin */
  GPIO_InitStruct.Pin = USB_C_P_Pin|USB_C_PA11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_CCK1_Pin */
  GPIO_InitStruct.Pin = MIC_CCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_CCK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WRLS_WKUP_B_Pin */
  GPIO_InitStruct.Pin = WRLS_WKUP_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WRLS_WKUP_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WRLS_NOTIFY_Pin Mems_INT_IIS2MDC_Pin USB_IANA_Pin */
  GPIO_InitStruct.Pin = WRLS_NOTIFY_Pin|Mems_INT_IIS2MDC_Pin|USB_IANA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_IO6_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_IO6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_UCPD_FLT_Pin Mems_ISM330DLC_INT1_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_FLT_Pin|Mems_ISM330DLC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTOSPI_R_CLK_P_Pin OCTOSPI_R_NCS_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_CLK_P_Pin|OCTOSPI_R_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WRLS_SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = WRLS_SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(WRLS_SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_UCPD_CC2_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_CC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_UCPD_CC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Mems_STSAFE_RESET_Pin WRLS_WKUP_W_Pin */
  GPIO_InitStruct.Pin = Mems_STSAFE_RESET_Pin|WRLS_WKUP_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_SDIN0_Pin */
  GPIO_InitStruct.Pin = MIC_SDIN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_SDIN0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
