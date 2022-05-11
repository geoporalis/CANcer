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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[] = {0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22};//, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00
uint8_t RxData[8];
volatile uint32_t Instance = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DEBUG_PRINT_LN("## FDCAN 1:");
  DEBUG_PRINT_LN("  pre %ld, nomts1 %ld, nomts2 %ld", hfdcan1.Init.NominalPrescaler, hfdcan1.Init.NominalTimeSeg1, hfdcan1.Init.NominalTimeSeg2);
  DEBUG_PRINT_LN("	msgram sdt filt address: %lx",  hfdcan1.msgRam.StandardFilterSA);
  DEBUG_PRINT_LN("	msgram ext filt address: %lx",  hfdcan1.msgRam.ExtendedFilterSA);
  DEBUG_PRINT_LN("	msgram fifo 0 address: %lx",    hfdcan1.msgRam.RxFIFO0SA);
  DEBUG_PRINT_LN("	msgram fifo 1 address: %lx",    hfdcan1.msgRam.RxFIFO1SA);

  DEBUG_PRINT_LN("\n## FDCAN 2:");
  DEBUG_PRINT_LN("  pre %ld, nomts1 %ld, nomts2 %ld", hfdcan2.Init.NominalPrescaler, hfdcan2.Init.NominalTimeSeg1, hfdcan2.Init.NominalTimeSeg2);
  DEBUG_PRINT_LN("	msgram sdt filt address: %lx",  hfdcan2.msgRam.StandardFilterSA);
  DEBUG_PRINT_LN("	msgram ext filt address: %lx",  hfdcan2.msgRam.ExtendedFilterSA);
  DEBUG_PRINT_LN("	msgram fifo 0 address: %lx",    hfdcan2.msgRam.RxFIFO0SA);
  DEBUG_PRINT_LN("	msgram fifo 1 address: %lx",    hfdcan2.msgRam.RxFIFO1SA);

  /* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x111;
  sFilterConfig.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  canPrintFilterConfig(hfdcan1.msgRam.StandardFilterSA);
  canPrintFilterConfig(hfdcan2.msgRam.StandardFilterSA);

  /* Configure global filter on both FDCAN instances:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_HIGH_PRIORITY_MSG, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_HIGH_PRIORITY_MSG, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx message Header */
  TxHeader.Identifier = 0x111;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;					//FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  /* Configure and enable Tx Delay Compensation, required for BRS mode.
     TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
     TdcFilter default recommended value: 0 */
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module on both FDCAN instances */
//  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
//  {
//    Error_Handler();
//  }
  canConfigurateNStart(&hfdcan1,1000000,1,66);
  canConfigurateNStart(&hfdcan2,1000000,1,66);

  DEBUG_PRINT_LN("Hello");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1�s transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    /* Retrieve Rx messages from RX FIFO0 */
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
	  Error_Handler();
	}

	if (hfdcan->Instance == FDCAN1)
	{
	  DEBUG_PRINT("fdcan 1;")
	}
	else /* FDCAN2 */
	{
	  DEBUG_PRINT("fdcan 2;")
	}

	DEBUG_PRINT(" fifo 0; msg_id %lx; ", RxHeader.Identifier);
	for(uint8_t i = 0; i<8; i++){
		DEBUG_PRINT("%02x:", RxData[i]);
	}
	DEBUG_PRINT_LN(" ");
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }

	if (hfdcan->Instance == FDCAN1)
	{
	  DEBUG_PRINT("fdcan 1;")
	}
	else /* FDCAN2 */
	{
	  DEBUG_PRINT("fdcan 2;")
	}

	DEBUG_PRINT(" fifo 1; msg_id %lx; ", RxHeader.Identifier);
	for(uint8_t i = 0; i<8; i++){
		DEBUG_PRINT("%02x:", RxData[i]);
	}
	DEBUG_PRINT_LN(" ");
  }
}

void HAL_FDCAN_HighPriorityMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{

	if (hfdcan->Instance == FDCAN1)
	{
	  DEBUG_PRINT("fdcan 1 ");
	}
	else /* FDCAN2 */
	{
	  DEBUG_PRINT("fdcan 2 ");
	}
	DEBUG_PRINT(": this was a HP message");

}

/**
  * @brief EXTI line detection callback
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	DEBUG_PRINT_LN("button pressed: Instance %ld", Instance);
  if (GPIO_Pin == B1_Pin)
  {
    /* Turn LED1 and LED4 off */

    /* Check which FDCAN instance is selected to send message */
    if (Instance == 1)
	{
      /* Add message to TX FIFO of FDCAN instance 1 */
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
      {
        Error_Handler();
      }

      /* Toggle FDCAN instance */
      Instance = 2;
    }
    else /* Instance = 2 */
    {
      /* Add message to TX FIFO of FDCAN instance 2 */
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData) != HAL_OK)
      {
        Error_Handler();
      }

      /* Toggle FDCAN instance */
      Instance = 1;
    }

    /* Delay to avoid rebound */
//    HAL_Delay(100);
  }
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
  DEBUG_PRINT_LN(" You reached the end of support ... bye")
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

