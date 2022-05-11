/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
can_baudrate_t can_baud;
uint32_t can_started = 0;
FDCAN_TxHeaderTypeDef tx_header;

void canConfigTXHeaderDefault();
void canConfigEvents(FDCAN_HandleTypeDef *hfdcan);
void canStart(FDCAN_HandleTypeDef *hfdcan);
void canStop(FDCAN_HandleTypeDef *hfdcan);
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
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
  hfdcan1.Init.NominalPrescaler = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 35;
  hfdcan1.Init.NominalTimeSeg2 = 20;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 3;
  hfdcan2.Init.NominalSyncJumpWidth = 8;
  hfdcan2.Init.NominalTimeSeg1 = 35;
  hfdcan2.Init.NominalTimeSeg2 = 20;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */
    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);

    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void canConfigurateNStart(FDCAN_HandleTypeDef *hfdcan, uint32_t baud, uint32_t sync_pzt, uint32_t sampl_pzt){



//		canPrintFilterConfig();

//	canConfigFiltersDefault(hfdcan);

//	DEBUG_PRINT_LN("		after config")

//		canPrintFilterConfig();


//	canConfigEvents(hfdcan);		// placeholder, not used

//	canConfigTXHeaderDefault();


	can_baud = (can_baudrate_t){
			.baud_rate = baud,
			.syncbit_percent = sync_pzt,
			.samplbit_percent = sampl_pzt,
	};

	canConfigBaudRate(hfdcan);
//	canStart();

}



void canConfigFiltersDefault(FDCAN_HandleTypeDef *hfdcan){

	FDCAN_FilterTypeDef sFilterConfig;


	/* Configure Rx filter */
		sFilterConfig.IdType = FDCAN_STANDARD_ID;				//FDCAN_EXTENDED_ID;				//
		sFilterConfig.FilterIndex = 0;
		sFilterConfig.FilterType = FDCAN_FILTER_MASK;			// FDCAN_FILTER_DUAL;			//FDCAN_FILTER_RANGE;	//FDCAN_FILTER_REJECT;		//
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;	// FDCAN_FILTER_TO_RXFIFO0_HP;	//FDCAN_FILTER_DISABLE;		//
		sFilterConfig.FilterID1 = 0x1BB;						// CAN_NODE_ADD_ON;			// 0;		0x000;// +
		sFilterConfig.FilterID2 = 0x0; 						    //0x1BB;//1FFFFFFF;// +CAN_NODE_ADD_ON;		//0x7FF;

		if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
		{
			DEBUG_PRINT_LN("Error in CAN filter configuration!!");
			return;
		}
	DEBUG_PRINT_LN(" ");
	DEBUG_PRINT_LN(" msgram sdt filt address: %lx", hfdcan->msgRam.StandardFilterSA);
	DEBUG_PRINT_LN(" msgram ext filt address: %lx", hfdcan->msgRam.ExtendedFilterSA);
	DEBUG_PRINT_LN(" msgram fifo 0 address: %lx", 	hfdcan->msgRam.RxFIFO0SA);
	DEBUG_PRINT_LN(" msgram fifo 1 address: %lx", 	hfdcan->msgRam.RxFIFO1SA);

	  /* Configure global filter:
	     Filter all remote frames with STD and EXT ID
	     Reject non matching frames with STD ID and EXT ID */
//,FDCAN_REJECT	FDCAN_REJECT_REMOTE	FDCAN_ACCEPT_IN_RX_FIFO1
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_REJECT, FDCAN_REJECT_REMOTE , FDCAN_REJECT_REMOTE ) != HAL_OK)	 //,//FDCAN_FILTER_REMOTE //FDCAN_REJECT_REMOTE	//FDCAN_REJECT
	{
		DEBUG_PRINT_LN("Error in CAN filter configuration!!");
		return;
	}

	return;
}


void canConfigBaudRate(FDCAN_HandleTypeDef *hfdcan){

	uint32_t div = (uint32_t)(170000000/can_baud.baud_rate);

	uint32_t sync_bit = (uint32_t)(div*0.01f*can_baud.syncbit_percent);

	uint32_t range = (uint32_t)(div/sync_bit)-1;

	uint32_t timeseg_1 = (uint32_t)(range*0.01f*can_baud.samplbit_percent);
	uint32_t timeseg_2 = (uint32_t)(range-timeseg_1);

	hfdcan->Init.NominalPrescaler = sync_bit;
	hfdcan->Init.NominalTimeSeg1 = timeseg_1;
	hfdcan->Init.NominalTimeSeg2 = timeseg_2;

//	if(can_started != 0 ){
//		canStop(hfdcan);
//	}
	if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
	{
		Error_Handler();
	}
	canStart(hfdcan);
}

void canConfigEvents(FDCAN_HandleTypeDef *hfdcan){
	// FIFO 0
	  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_FULL, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  // FIFO 1
	  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  // ELSE
	  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RAM_ACCESS_FAILURE, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

/**
 * start CAN periphery
 * @return see global.h
 */
void canStart(FDCAN_HandleTypeDef *hfdcan){

	if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
	{
		DEBUG_PRINT_LN("Error to start CAN periphery");
		can_started = 0;
	}
	can_started = 1;
}

/**
 * stop CAN periphery
 * @return see global.h
 */
void canStop(FDCAN_HandleTypeDef *hfdcan){

	if (HAL_FDCAN_Stop(hfdcan) != HAL_OK)
	{
		DEBUG_PRINT_LN("Error to stop CAN periphery");
	}

	can_started = 0;
}

void canConfigTXHeaderDefault(){

	tx_header.Identifier = 0x111;
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = FDCAN_DLC_BYTES_8;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;

}


/**
 * Add a message to the Tx FiFo/Queue
 * @param tx_id .. message identifier
 * @param tx_data .. message data
 * @return see global.h
 */
void canTransmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *tx_data, uint8_t len){

	tx_header.Identifier = id;
	tx_header.IdType = FDCAN_STANDARD_ID;

	tx_header.DataLength = len;

	HAL_StatusTypeDef msg = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
	if (msg != HAL_OK)
	{
		DEBUG_PRINT_LN("Error in CAN TX: %lu %lx,", (uint32_t)msg,  (uint32_t)hfdcan->ErrorCode);	//");
	}

}


void canPrintFilterConfig(uint32_t address){

	  uint32_t *FilterAddress = (uint32_t*)address;
	  DEBUG_PRINT_LN("Std: Address: %lx, value: %lx : SFT: %lx SFEC: %lx SFID1: %lx SFID1: %lx", (uint32_t)FilterAddress, *(uint32_t*)FilterAddress
			  	  ,  (*(uint32_t*)FilterAddress)>>30
				  , ((*(uint32_t*)FilterAddress)>>27) & 0x7
				  , ((*(uint32_t*)FilterAddress)>>16) & 0x7FF
				  , ((*(uint32_t*)FilterAddress)>> 0) & 0x7FF
				  );

	  FilterAddress++;
	  DEBUG_PRINT_LN("Std: Address: %lx, value: %lx : SFT: %lx SFEC: %lx SFID1: %lx SFID1: %lx", (uint32_t)FilterAddress, *(uint32_t*)FilterAddress
				  ,  (*(uint32_t*)FilterAddress)>>30
				  , ((*(uint32_t*)FilterAddress)>>27) & 0x7
				  , ((*(uint32_t*)FilterAddress)>>16) & 0x7FF
				  , ((*(uint32_t*)FilterAddress)>> 0) & 0x7FF
				  );

}















/* USER CODE END 1 */
