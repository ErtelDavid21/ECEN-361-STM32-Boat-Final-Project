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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LORA_RX_TIMEOUT 2000  // ms
#define LORA_BUFFER_SIZE 256
#define UART_DELAY 100 // wait max of 100 ms between frames in message
#define MAX_MESSAGE_SIZE 100 // 100 characters maximum message size
#define LORA_RX_BUF_LEN 128

#define ADC_MAX_VALUE    4095      // 12-bit ADC
#define PWM_MAX_VALUE    50		   // 50Hz duty cycle

// Choose which node this board will be
//#define NODE_ALPHA   1
#define NODE_BETA  1

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t message[MAX_MESSAGE_SIZE] = { 0 }; // char array to store message received
uint8_t response[MAX_MESSAGE_SIZE] = { 0 }; // char array to store response message
uint8_t uart1_byte; // byte received from UART1
uint8_t uart2_byte; // byte received from UART2
uint8_t buffer_position = 0; // how many bytes received so far in message
uint16_t LoRa_RxIndex = 0;
volatile char lora_rx_buffer[LORA_RX_BUF_LEN];
volatile uint16_t lora_rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
bool LoRa_SendCommand(UART_HandleTypeDef *huart, const char *cmd, char *response_buf, uint16_t buf_len);
bool LoRa_Init(UART_HandleTypeDef *huart, uint16_t address);
bool LoRa_SendMessage(UART_HandleTypeDef *huart, uint16_t destAddr, const char *msg);
//int LoRa_ReadMessage(UART_HandleTypeDef *huart);
bool LoRa_ReadLine(UART_HandleTypeDef *huart, char *buffer, uint16_t max_len, uint32_t timeout);
void LoRa_StartReceive_IT(UART_HandleTypeDef *huart);
void LoRa_ProcessLine(char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  HAL_Delay(1000); // wait for modules to boot

  printf("Initializing LoRa module...\r\n");

  #ifdef NODE_ALPHA
  HAL_UART_AbortReceive_IT(&huart1);  // Stop interrupt
  LoRa_Init(&huart1, 1);              // Do blocking init
  LoRa_StartReceive_IT(&huart1);      // Re-enable interrupt mode

  #endif

  #ifdef NODE_BETA
  LoRa_Init(&huart3, 2);  // Beta has address 2
  #endif

  printf("LoRa init complete.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	       Beta node: send-only
	      if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	      {
	          HAL_Delay(50);  // debounce

	          // Optional: short wait to ensure LoRa module is ready
	          HAL_Delay(10);

	          // Flush UART to clear any stray data
	          __HAL_UART_FLUSH_DRREGISTER(&huart3);
	          __HAL_UART_CLEAR_OREFLAG(&huart3);
	          __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_NEF);

//	           Send message to Alpha (address 1)
	          LoRa_SendMessage(&huart3, 1, "LEFT");


	          // Wait for button release to avoid multiple sends
	          while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET);
	      }

	          // LED feedback
	          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	      HAL_Delay(100);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  sConfig.Channel = ADC_CHANNEL_11;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, RLED_Pin|YLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GLED_Pin|BLED_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RLED_Pin YLED_Pin */
  GPIO_InitStruct.Pin = RLED_Pin|YLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GLED_Pin BLED_Pin LD2_Pin */
  GPIO_InitStruct.Pin = GLED_Pin|BLED_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)  // UART1 → LoRa Alpha
    {
        char ch = uart1_byte;

        // Store byte
        if (lora_rx_index < LORA_RX_BUF_LEN - 1) {
            lora_rx_buffer[lora_rx_index++] = ch;
        }

        // Check for line ending
        if (ch == '\n' && lora_rx_index >= 2 &&
            lora_rx_buffer[lora_rx_index - 2] == '\r')
        {
            lora_rx_buffer[lora_rx_index - 2] = '\0'; // terminate string
            printf("LoRa Line: %s\r\n", lora_rx_buffer);

            // Process it immediately
            LoRa_ProcessLine((char*)lora_rx_buffer);

            lora_rx_index = 0; // reset for next line
        }

        // Continue receiving next byte
        HAL_UART_Receive_IT(&huart1, &uart1_byte, 1);
    }
}

void LoRa_ProcessLine(char *line)
{
    if (strstr(line, "+RCV=") == line)
    {
        int sender, len, rssi, snr;
        char msg[64] = {0};

        if (sscanf(line, "+RCV=%d,%d,%[^,],%d,%d", &sender, &len, msg, &rssi, &snr) == 5)
        {
            msg[len] = '\0';
            printf("\r\n--- LoRa RX ---\r\n");
            printf("  From: %d | RSSI: %d, SNR: %d\r\n", sender, rssi, snr);
            printf("  Payload: [%s]\r\n", msg);

            if (strcasecmp(msg, "FORWARD") == 0) {

                HAL_GPIO_TogglePin(GLED_GPIO_Port, GLED_Pin);
            }
            else if (strcasecmp(msg, "STOP") == 0) {

                HAL_GPIO_TogglePin(RLED_GPIO_Port, RLED_Pin);
            }
            else if (strcasecmp(msg, "RIGHT") == 0) {

                HAL_GPIO_TogglePin(YLED_GPIO_Port, YLED_Pin);
            }
            else if (strcasecmp(msg, "LEFT") == 0) {

                HAL_GPIO_TogglePin(BLED_GPIO_Port, BLED_Pin);
            }
            else
                printf("  >> Unrecognized command\r\n");
        }
    }
}

// Send AT command and wait for a +OK or +RCV response
bool LoRa_SendCommand(UART_HandleTypeDef *huart, const char *cmd, char *response_buf, uint16_t buf_len)
{
    char cmd_with_crlf[128];
    char rx_line[128];

    // Print command to console for debugging
    printf("-> %s\r\n", cmd);

    // 1. Send the command (must end with \r\n)
    snprintf(cmd_with_crlf, sizeof(cmd_with_crlf), "%s\r\n", cmd);
    HAL_UART_Transmit(huart, (uint8_t*)cmd_with_crlf, strlen(cmd_with_crlf), HAL_MAX_DELAY);

    // 2. Wait for a response (it might be +OK, or the requested data)
    // The module can return multiple lines (+<value> then +OK)
    uint32_t startTick = HAL_GetTick();

    // Loop to read all lines until timeout or +OK is found
    while (HAL_GetTick() - startTick < LORA_RX_TIMEOUT)
    {
        if (LoRa_ReadLine(huart, rx_line, sizeof(rx_line), LORA_RX_TIMEOUT / 4))
        {
            printf("<- %s\r\n", rx_line);

            // Check for success (+OK)
            if (strstr(rx_line, "+OK") != NULL)
            {
                return true;
            }
            // Check for error (+ERR)
            else if (strstr(rx_line, "+ERR") != NULL)
            {
                printf("LoRa command failed: %s\r\n", cmd);
                return false;
            }
            // Capture any non-status line (like AT+ADDRESS? response)
            else if (response_buf != NULL)
            {
                strncpy(response_buf, rx_line, buf_len - 1);
                response_buf[buf_len - 1] = '\0';
                // Note: We continue to wait for +OK after capturing the value.
            }
        }
    }

    // If timeout occurred without +OK or +ERR
    printf("LoRa command timed out: %s\r\n", cmd);
    return false;
}

bool LoRa_ReadLine(UART_HandleTypeDef *huart, char *buffer, uint16_t max_len, uint32_t timeout)
{
    uint32_t start = HAL_GetTick();
    uint16_t pos = 0;
    char c;

    while (HAL_GetTick() - start < timeout)
    {
        if (HAL_UART_Receive(huart, (uint8_t*)&c, 1, 5) == HAL_OK)
        {
            if (c == '\n') {
                buffer[pos] = '\0';
                return true;
            }

            if (pos < max_len - 1) {
                buffer[pos++] = c;
            }
        }
    }

    return false; // timeout
}

// Initialize LoRa module
bool LoRa_Init(UART_HandleTypeDef *huart, uint16_t address)
{
    char cmd[64];
    bool success = true;

    // ---- Step 1: Flush and wait for module boot noise ----
    __HAL_UART_FLUSH_DRREGISTER(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_Delay(500); // allow any "+READY" lines to finish

    // ---- Step 2: Clear out any leftover startup text ----
    char dummy[64];
    while (LoRa_ReadLine(huart, dummy, sizeof(dummy), 100))
    {
        printf("LoRa boot: %s\r\n", dummy); // optional
    }

    // ---- Step 3: Ping the module with AT ----
    HAL_Delay(50); // small extra delay
    success &= LoRa_SendCommand(huart, "AT", NULL, 0);
    if (!success)
    {
        printf("⚠️  Warning: AT command timeout, retrying once...\r\n");
        HAL_Delay(200);
        success = LoRa_SendCommand(huart, "AT", NULL, 0);
    }

    if (!success)
    {
        printf("❌ LoRa not responding after retry.\r\n");
        return false;
    }

    // ---- Step 4: Apply settings ----
    snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%u", address);
    success &= LoRa_SendCommand(huart, cmd, NULL, 0);
    success &= LoRa_SendCommand(huart, "AT+NETWORKID=10", NULL, 0);
    success &= LoRa_SendCommand(huart, "AT+BAND=915000000", NULL, 0);
    success &= LoRa_SendCommand(huart, "AT+PARAMETER=7,7,1,4", NULL, 0);

    if (success)
    {
        printf("✅ LoRa Module on %s initialized to Address %u.\r\n",
               (huart == &huart1) ? "UART1 (Alpha)" : "UART3 (Beta)", address);
    }

    return success;
}

// Send message to destination node
bool LoRa_SendMessage(UART_HandleTypeDef *huart, uint16_t destAddr, const char *msg)
{
    char cmd[128];
    int len = strlen(msg);

    // Command format: AT+SEND=<Address>,<Length>,<Data>
    snprintf(cmd, sizeof(cmd), "AT+SEND=%u,%d,%s", destAddr, len, msg);

    // AT+SEND should return +OK upon transmission success
    return LoRa_SendCommand(huart, cmd, NULL, 0);
}

void LoRa_StartReceive_IT(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(huart, &uart1_byte, 1);

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
