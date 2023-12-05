/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "debug.h"
#include "torkTypes.h"
#include "basicApp.h"
#include "mcp2515_hardware.h"
#include "mcp2515_drv.h"
#include "string.h"
#include "stdbool.h"
#include "canMsgRingBuf.h"
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
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*****************************************************************************************************************************************/
// for led blink
uint32_t period_of_led_blink = _1_SECOND;
uint32_t last_time = 0;
/*****************************************************************************************************************************************/


/*****************************************************************************************************************************************/
// for CanA comm
FDCAN_TxHeaderTypeDef txHeader_A;  //CAN Bus Receive Header
FDCAN_RxHeaderTypeDef rxHeader_A;  //CAN Bus Receive Header
uint8_t bufRx_A[8] ={ 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN Bus Receive Buffer

// for CanB comm
FDCAN_TxHeaderTypeDef txHeader_B;  //CAN Bus Receive Header
FDCAN_RxHeaderTypeDef rxHeader_B;  //CAN Bus Receive Header
uint8_t bufRx_B[8] ={ 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN Bus Receive Buffer
/*****************************************************************************************************************************************/


/***************************************************************************************************/
torkCanMsg tempCanMsg_A;    //canA Rx den alınan mesajları Ringbuffera atmak icin kullanılır
torkCanMsg tempCanMsg_B;    //canB Rx den alınan mesajları Ringbuffera atmak icin kullanılır
torkCanMsg tempCanMsg_C;    //canC Rx den alınan mesajları Ringbuffera atmak icin kullanılır

torkCanMsg tempCanMsgTx;

uint8_t result     = 0;    //canC rx de
uint8_t result_One = 0;    //route1 poplamada kullanıldı
uint8_t result_Two = 0;    //route2 poplamada kullanıldı
/***************************************************************************************************/


/***************************************************************************************************/
// for CanC comm
volatile uint8_t isCAN_C_RXed = 0;
extern CanbusConfig_t cfgCanC;   //spi2
/***************************************************************************************************/

/***************************************************************************************************/
extern Can_Route_Values_t routeOne;
extern Can_Route_Values_t routeTwo;
/***************************************************************************************************/

/***************************************************************************************************/
// for debug
char failMsg[] = "CAN_FAIL\n"; //
char okMsg[]   = "CAN_OK\n";
/***************************************************************************************************/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
/* USER CODE BEGIN PFP */
void heartBeat();
void isPressedBtn();
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */

  Init_Basic_App();

  tempCanMsg_B.FrameType           = FDCAN_DATA_FRAME;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	   *
	   * routeOne :  canA ------|------> canB
	   * 			 canC ------|
	   *
	   * canA ve canC'ye veri geldiginde route1'e koy    (stage1)
	   * route1 de veri varsa canB'den gönder            (stage2)
	   */

	  /*
	   *
	   * routeTwo :  canB -------|------> canA
	   * 						 |------> canC
	   *
	   * canB veri geldiginde route2'e koy               (stage1)
	   * route2 de veri varsa canA ve can C'den gönder   (stage2)
	   */

		/*****************************************************************************************************************************************/
		/*                                                       STAGE 1 IRQ KONTROLU BASLA                                                      */
		/*****************************************************************************************************************************************/

		/**********************************************************************************/
		/*CAN A ve CAN C PORTUNA VERI GELDI MI         "CALLBACK ICERISINDE"              */
		/**********************************************************************************/

		/**********************************************************************************/
		/*CAN C PORTUNA VERI GELDI MI                                                     */
		/**********************************************************************************/
		if (isCAN_C_RXed == 1)
		{
			isCAN_C_RXed = 0;
			HAL_GPIO_WritePin(LED_CAN_C_RX_GPIO_Port , LED_CAN_C_RX_Pin, GPIO_PIN_SET);

			while (CAN_MSGAVAIL == checkReceive(&cfgCanC))
			{
				uint8_t length;
				result = readMsgBuf(&cfgCanC, &length, tempCanMsg_C.Payload);
				if (result == CAN_OK)
				{
					tempCanMsg_C.DataLength    = Convert_Can_Length_For_StmLib(length);
					tempCanMsg_C.Identifier    = cfgCanC.can_id;
					cfgCanC.rtr    ?  (tempCanMsg_C.FrameType = FDCAN_REMOTE_FRAME) : (tempCanMsg_C.FrameType = FDCAN_DATA_FRAME);
					cfgCanC.ext_flg ? (tempCanMsg_C.IdType = FDCAN_EXTENDED_ID)     : (tempCanMsg_C.IdType = FDCAN_STANDARD_ID);

					debugPrint("RX, ->canC  ->route1\n");
					canMsgRingBufferPush(&routeOne.Route_Ring_Buf, tempCanMsg_C);

				}
				else
				{
					debugPrintf(failMsg);
				}
			}
		}
		/*****************************************************************************************************************************************/
		/*                                                       STAGE 1 IRQ KONTROLU  SON                                                       */
		/*****************************************************************************************************************************************/




		/*****************************************************************************************************************************************/
		/*                                                       STAGE 2 DATA ISLEM    BASLA                                                     */
		/*****************************************************************************************************************************************/

		result_One = canMsgRingBufferPop(&routeOne.Route_Ring_Buf, &tempCanMsgTx);

		while (result_One == CAN_OK)
		{
			_Bool compResult = Compare_Is_Incoming_Message_Different_From_Previous_Message(tempCanMsgTx, routeOne.Last_Sent_Message);
			routeOne.Last_Sent_Message = tempCanMsgTx;

//			_Bool compResult = Compare_Is_Incoming_Message_Different_From_Previous_Two_Message(canMsgTx, routeOne.Penultimate_Sent_Message, routeOne.Last_Sent_Message);
//			routeOne.Penultimate_Sent_Message = routeOne.Last_Sent_Message;
//			routeOne.Last_Sent_Message = canMsgTx;

			if (compResult)
			{
				debugPrint("TX, route1-> canB->\n");


				txHeader_B.TxFrameType  = tempCanMsgTx.FrameType;
				txHeader_B.DataLength   = tempCanMsgTx.DataLength;


				if(tempCanMsgTx.IdType == FDCAN_STANDARD_ID)
				{
					txHeader_B.IdType     = FDCAN_EXTENDED_ID;
					txHeader_B.Identifier = ((tempCanMsgTx.Identifier << 18) + tempCanMsgTx.Payload[0]);          // Shift the 11-bit identifier to the left by 18 bits to make it a 29-bit identifier
				}
				else
				{
					txHeader_B.IdType     = FDCAN_EXTENDED_ID;
					txHeader_B.Identifier = tempCanMsgTx.Identifier;
				}

				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader_B, tempCanMsgTx.Payload);  //canB den gönder

			}

			result_One = canMsgRingBufferPop(&routeOne.Route_Ring_Buf, &tempCanMsgTx);
			//ledler
//			HAL_GPIO_WritePin(LED_CAN_B_TX_GPIO_Port, LED_CAN_B_TX_Pin, GPIO_PIN_SET);//... tx leder? nasıl söncek?
			HAL_GPIO_WritePin(LED_CAN_A_RX_GPIO_Port, LED_CAN_A_RX_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_CAN_C_RX_GPIO_Port, LED_CAN_C_RX_Pin, GPIO_PIN_SET);
		}



		/*****************************************************************************************************************************************/
				/*ROUTE 2 RINGBUFF VE PORT YONLENDIRME*/
		/*****************************************************************************************************************************************/
				result_Two = canMsgRingBufferPop(&routeTwo.Route_Ring_Buf, &tempCanMsgTx);

				while (result_Two == CAN_OK)
				{
					debugPrint("TX, route2-> canA & canC->\n");

						//CanA icin
					if(tempCanMsgTx.IdType == FDCAN_EXTENDED_ID)
					{
						txHeader_A.IdType = FDCAN_STANDARD_ID;
						txHeader_A.Identifier = tempCanMsgTx.Identifier >> 18;  //? başka bir şey
					}
					else
					{
						txHeader_A.IdType = FDCAN_STANDARD_ID;
						txHeader_A.Identifier  = tempCanMsgTx.Identifier;
					}

					//txHeader_A.TxFrameType = tempCanMsgTx.FrameType;    //gerek yok canA init de atandı zaten, mainScp remote frame de sapıttığı icin hep data frame gönderilir
					txHeader_A.DataLength  = tempCanMsgTx.DataLength;

					HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader_A, tempCanMsgTx.Payload);  //canB den gönder


						//Canc icin
					if(tempCanMsgTx.IdType == FDCAN_EXTENDED_ID)
					{
						tempCanMsgTx.IdType = 0;
						tempCanMsgTx.Identifier = tempCanMsgTx.Identifier >> 18;  //? başka bir şey
					}
					else
					{
						tempCanMsgTx.IdType = 0;
					}

					//tempCanMsgTx.FrameType   = 0;   //gerek yok canB rx callback de atandı zaten, mainScp remote frame de sapıttığı icin hep data frame gönderilir

					sendMsgBuffer(&cfgCanC, tempCanMsgTx.Identifier, (uint8_t)tempCanMsgTx.IdType, (uint8_t)tempCanMsgTx.FrameType, Convert_Can_Length_For_McpLib(tempCanMsgTx.DataLength), tempCanMsgTx.Payload);

				   // memset(tempCanMsgTx.Payload, 0, MAX_CAN_MSG_DATA_COUNT);  //payload sıfırla
					result_Two = canMsgRingBufferPop(&routeTwo.Route_Ring_Buf, &tempCanMsgTx);

					HAL_GPIO_WritePin(LED_CAN_B_RX_GPIO_Port, LED_CAN_B_RX_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED_CAN_A_TX_GPIO_Port, LED_CAN_A_TX_Pin, GPIO_PIN_SET);   //... söndürmeyi düşün
					HAL_GPIO_WritePin(LED_CAN_A_TX_GPIO_Port, LED_CAN_A_TX_Pin, GPIO_PIN_SET);  //...
				}

		/*****************************************************************************************************************************************/
		/*                                                       STAGE 2 DATA ISLEM    SON                                                       */
		/*****************************************************************************************************************************************/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*****************************************************************************************************************************************/
		  	  	  	  	  	  	  	  	  	  	  /*BUTON CHECK VE BLINK*/
	  /*****************************************************************************************************************************************/

		  isPressedBtn();
		  heartBeat();

	  /*****************************************************************************************************************************************/
	  /*****************************************************************************************************************************************/
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 27;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 27;
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
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
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
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1 = 27;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 4;
  hfdcan2.Init.DataTimeSeg1 = 27;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_CAN_A_RX_Pin|LED_CAN_A_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_CAN_B_TX_Pin|LED_CAN_B_RX_Pin|LED_CAN_C_RX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_CAN_C_Pin|RST_CANC_IC_Pin|LED_BLINK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_CAN_C_TX_GPIO_Port, LED_CAN_C_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_CAN_A_RX_Pin LED_CAN_A_TX_Pin */
  GPIO_InitStruct.Pin = LED_CAN_A_RX_Pin|LED_CAN_A_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_CAN_B_TX_Pin LED_CAN_B_RX_Pin LED_CAN_C_RX_Pin */
  GPIO_InitStruct.Pin = LED_CAN_B_TX_Pin|LED_CAN_B_RX_Pin|LED_CAN_C_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_CAN_C_Pin RST_CANC_IC_Pin LED_BLINK_Pin */
  GPIO_InitStruct.Pin = CS_CAN_C_Pin|RST_CANC_IC_Pin|LED_BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_CAN_C_Pin */
  GPIO_InitStruct.Pin = INT_CAN_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_CAN_C_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_CAN_C_TX_Pin */
  GPIO_InitStruct.Pin = LED_CAN_C_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_CAN_C_TX_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * function   :   heartBeat():  cihazın calisma durumunu gösterir
 * 				  				1 sn  aralıklarla yanması      NORMAL
 * 				  				200ms aralıklarla yanması      ARIZA  //... farklı arıza durumlarında farklı period koy
 * 				  				loop icerisinde cagrılmalıdır. "period_of_led_blink" degiskeni ile frekansı belirlenir.
 * parameters :   void
 * return     :   void
*/
void heartBeat()
{
	if(HAL_GetTick() - last_time > period_of_led_blink)
	{
		last_time = HAL_GetTick();
		HAL_GPIO_TogglePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin);
	}
}


/**
 * function   :  isPressedBtn0(): switch 1 basılmasını algılar
 * 				 Butona basmayı algılaması icin loop icerisinde sürekli cagrılmalıdır!
 *
 * parameters :  void
 *
 * return     :  void
 **/
void isPressedBtn()
{
	static int msCount = 0;

	if(!HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin))
	{
		msCount++;
		if(msCount > 1e3)
		{

			//***********************************
			// do something
			//***********************************
			debugPrint("btn\n");

			uint8_t dataBUFFFER[8] = {1, 2, 3, 4, 5, 6, 7, 8};

			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader_B, dataBUFFFER);  //canB den gönder

			//***********************************

			msCount = -3e6;

		}
	}
	else
	{
		msCount = 0;
	}
}


/**
 * CAN_A 'a gelen mesajlarda hfdcan1'ın FIFO0'ına konur  line0 kesmesi devreye girer
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader_A, tempCanMsg_A.Payload) == HAL_OK)
		{
			//dbgPrint("canA new RxFifo0 :"); dbgDumpHex( tempCanMsg_A.Payload, 8); dbgPrint("\n");
			//dbgPrintf("RxHeader.Identifier: %x\n", rxHeader_A.Identifier);

			HAL_GPIO_WritePin(LED_CAN_A_RX_GPIO_Port, LED_CAN_A_RX_Pin, GPIO_PIN_SET);

			tempCanMsg_A.Identifier          = rxHeader_A.Identifier;
			tempCanMsg_A.IdType              = rxHeader_A.IdType;           // mainscp'den hep std ide gelir
			tempCanMsg_A.FrameType           = rxHeader_A.RxFrameType;      // mainscp'den hep data frame gelir
			tempCanMsg_A.DataLength          = rxHeader_A.DataLength;

			canMsgRingBufferPush(&routeOne.Route_Ring_Buf, tempCanMsg_A);

			debugPrint("RX, ->canA  ->route1\n");
		}
		//dbgPrint("ERROR: Fifo0Callback HAL_FDCAN_GetRxMessage\n");
		//Error_Handler();
	}
}


/**
 *  CAN_B 'a gelen mesajlarda hfdcan2'ın FIFO1'ına konur  line1 kesmesi devreye girer
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxHeader_B, tempCanMsg_B.Payload) == HAL_OK)
		{
			//dbgPrint("canB new RxFifo1 :"); dbgDumpHex(tempCanMsg_B.Payload, 8); dbgPrint("\n");
			//dbgPrintf("RxHeader.Identifier: %x\n", rxHeader_B.Identifier);

			HAL_GPIO_WritePin(LED_CAN_B_RX_GPIO_Port, LED_CAN_B_RX_Pin,GPIO_PIN_SET);

			tempCanMsg_B.Identifier          = rxHeader_B.Identifier;
			tempCanMsg_B.IdType              = rxHeader_B.IdType;
			//tempCanMsg_B.FrameType           = FDCAN_DATA_FRAME;   //...   mainScp kartı remote frame aldıgında sapıtıyor, whil(1) den önce ataması yapıldı
			tempCanMsg_B.DataLength          = rxHeader_B.DataLength;

			canMsgRingBufferPush(&routeTwo.Route_Ring_Buf, tempCanMsg_B);

			debugPrint("RX, ->canB  ->route2\n");
		}
		//dbgPrint("ERROR: Fifo0Callback HAL_FDCAN_GetRxMessage\n");
		//Error_Handler();
	}
}


void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
		isCAN_C_RXed = 1;
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
