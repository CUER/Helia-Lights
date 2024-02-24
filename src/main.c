/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>

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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader; // CAN Transmission Header
CAN_RxHeaderTypeDef RxHeader; // CAN Receive Header
uint8_t TxData[8]; // CAN Transmission Data
uint8_t RxData[8]; // CAN Receive Data
uint32_t TxMailbox; // CAN Transmission Mailbox

char uart_msg_buf[100]; // buffer to store UART messages for transmission
int msg_len; // length of UART message
uint8_t tick_count = 0; // used in UART messages for ease of debugging. wraps quickly.

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  msg_len = sprintf(uart_msg_buf, "Lights Board Test. Listening for CAN messages...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // /* USER CODE END WHILE */

    // // the 2 bytes of data being sent
    // TxData[0] = 50;  
    // TxData[1] = 0xAA;

    // msg_len = sprintf(uart_msg_buf, "Sending CAN message %d...\r\n", tick_count);
    // HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);

    // if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    //   msg_len = sprintf(uart_msg_buf, "ERROR! %d", (int)hcan1.ErrorCode);
    //   HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);

    //   Error_Handler();
    // }

    // HAL_Delay(1000);
    // tick_count++;

    // /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{
  // Transmission details
  TxHeader.IDE = CAN_ID_STD; // using standard ID
  TxHeader.RTR = CAN_RTR_DATA; // indicates we are sending dataframe?
  TxHeader.StdId = 0x446; // ID of transmitter (us)
  TxHeader.DLC = 2; // length of data being sent in bytes

  // CAN configuration
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE; // if this is ENABLED it crashes after 3 msg. if DISABLED it crashes after 20 msg.
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  // initialise CAN
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  // filter configuration
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Can be FIFO0 or FIFO1 
  sFilterConfig.SlaveStartFilterBank = 13; // Meaningless for devices with one CAN peripheral (like ours). devices with one CAN peripheral have 13 filter banks per FIFO.
  sFilterConfig.FilterBank = 0; // Meaningless for devices with one CAN peripheral. Can be any value from 0-13.
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // Using 2x 16-bit filters instead of one 32-bit
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; // In mask mode, both the FilterId and FilterMask are used to decide whether to interrupt. In list mode, only FilterId is used.
  // 32-bit FilterScale: The FilterId high&low combine to form the full address. Long CAN addresses are 29 bits, so leftshift low part by 3 bits, and high part correspondingly. https://community.st.com/t5/stm32-mcus-embedded-software/can-setup-and-filters-using-hal/td-p/353986
  // 16-bit FilterScale: FilterIdHigh and FilterIdLow act as two independent 16-bit filters. Since short CAN addresses are 11 bits, and 5 MSBs are taken, left-shift address by 5 bits. 
  sFilterConfig.FilterIdHigh = 0x730<<5;
  sFilterConfig.FilterIdLow = 0x543<<5;
  sFilterConfig.FilterMaskIdHigh = 0x0000; // irrelevant when using FilterMode IDLIST
  sFilterConfig.FilterMaskIdLow = 0x0000; // irrelevant when using FilterMode IDLIST
  sFilterConfig.FilterActivation = ENABLE;

  // filter configuration
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // begin CAN
  if(HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  // enables interrupts when receiving CAN messages
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA8 PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
GPIO_PinState bool2gpio(uint8_t b) {
  return b ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

// LIGHTS FUNCTIONS
// call these to turn on/off the lights. the names are as defined on the lights board
// https://drive.google.com/drive/u/2/folders/1wjnNCMAwkKEKZ1ogMoTRsJ6xaBUVCbfb
// GPIO_PIN_SET to turn on, GPIO_PIN_RESET to turn off
// Each function remembers the state of the light. If the software state of the light changes,
// it tells the board to update the light's state IRL.
void set_RINDIC(GPIO_PinState set) { // PA_9
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_RINDIC was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_LINDIC(GPIO_PinState set) { // PA_10
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
      set_state = set;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, set);
    }
  msg_len = sprintf(uart_msg_buf, "Set_LINDIC was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_BRAKE(GPIO_PinState set) { // PB_0
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_BRAKE was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_REARPOS(GPIO_PinState set) { // PB_7
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_REARPOS was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_REVERSE(GPIO_PinState set) { // PB_6
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_REVERSE was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_REARFOG(GPIO_PinState set) { // PB_1
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_REARFOG was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_FRONTDAY(GPIO_PinState set) { // PA_1
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_FRONTDAY was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_FRONTHIGH(GPIO_PinState set) { // PA_8
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_FRONTHIGH was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_FRONTFOG(GPIO_PinState set) { // PA_0
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_FRONTFOG was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_HORN(GPIO_PinState set) { // PB_5
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_HORN was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_SAFELIGHT(GPIO_PinState set) { // PB_4
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_SAFELIGHT was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}
void set_FRONTFULL(GPIO_PinState set) { // PB_3
  static GPIO_PinState set_state = GPIO_PIN_RESET;
  if (set != set_state) {
    set_state = set;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, set);
  }
  msg_len = sprintf(uart_msg_buf, "Set_FRONTFULL was called and resulted in state %d...\r\n", set);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
}

void update_all_lights(uint8_t* DataPtr) {
  // documentation, address 0x730: https://docs.google.com/document/d/1VCUm64OMn_YMRYT_UJC2a-zEFHc4QdLCQOyA_3ngiRE/edit

  const uint8_t bit_mask = 0b00000001;
  uint8_t byte0 = DataPtr[0];
  set_BRAKE(bool2gpio(byte0 & bit_mask));
  set_REARFOG(bool2gpio((byte0>>1) & bit_mask));
  set_FRONTFOG(bool2gpio((byte0>>2) & bit_mask));
  set_FRONTDAY(bool2gpio((byte0>>3) & bit_mask)); // are these both front and back day lights?
  set_REARPOS(bool2gpio((byte0>>4) & bit_mask)); // what is this?
  set_REVERSE(bool2gpio((byte0>>5) & bit_mask));
  set_FRONTFULL(bool2gpio((byte0>>6) & bit_mask)); // what is this?
  set_FRONTHIGH(bool2gpio((byte0>>7) & bit_mask)); // what is this?
  uint8_t byte1 = DataPtr[1];
  set_RINDIC(bool2gpio(byte1 & bit_mask)); // right indicator
  set_LINDIC(bool2gpio((byte1>>1) & bit_mask)); // left indicator
  set_HORN(bool2gpio((byte1>>2) & bit_mask)); // is this HAZARD in the documentation? I ASSUMED THIS. CHECK.
  set_SAFELIGHT(bool2gpio((byte1>>3) & bit_mask)); // this is mounted on the inside of the car. 
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  /* Display LEDx */
  // if ((RxHeader.StdId == 0x321) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
  // {
  //   LED_Display(RxData[0]);
  // }
  msg_len = sprintf(uart_msg_buf, "Received CAN message:\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
  msg_len = sprintf(uart_msg_buf, (char*)RxData);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);
  msg_len = sprintf(uart_msg_buf, "\r\n\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_buf, msg_len, 100);

  // Decode CAN message and update lights
  update_all_lights(RxData);
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
