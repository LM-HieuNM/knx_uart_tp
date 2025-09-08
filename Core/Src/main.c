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
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    KNX_IDLE = 0,      // Chờ Start bit
    KNX_START = 1,     // Nhận Start bit
    KNX_DATA = 2,      // Nhận Data bits
    KNX_PARITY = 3,    // Nhận Parity bit
    KNX_STOP = 4       // Nhận Stop bit
} knx_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIT_US   	104     // tổng thời gian 1 bit
#define T0_HIGH   	35     // thời gian mức High trong bit 0
#define T0_LOW    	69     // thời gian mức Low trong bit 0
#define T0_TOL    	10   // dung sai

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t dma_buf[256];  // buffer cho DMA (high/low durations)
int dma_len = 0;

volatile uint32_t last_capture = 0;
volatile uint8_t  half_seen = 0;
volatile uint8_t  rx_bitcount = 0;
volatile uint8_t  rx_byte = 0;
volatile uint8_t  rx_frame[32];
volatile uint8_t  rx_index = 0;
volatile uint8_t  frame_ready = 0;
volatile uint8_t  waiting_for_bit = 0;  // đang chờ nhận bit
volatile uint32_t last_bit_time = 0;    // thời gian bit cuối
volatile uint8_t  frame_timeout = 0;    // timeout frame
volatile knx_state_t knx_state = KNX_IDLE;  // trạng thái KNX hiện tại
volatile uint8_t  data_bits = 0;        // số bit dữ liệu đã nhận
volatile uint8_t  parity_bit = 0;       // bit parity
volatile uint8_t  parity_count = 0;     // đếm số bit 1 trong data
volatile uint32_t rx_last_edge = 0;
volatile uint8_t rx_half_seen = 0;
/**
 *  ARR = 104 (chu kỳ 104 µs).
 *  CCR = 0 → PWM duty = 0% → Low suốt 104 µs (bit 1).
 *  CCR = 35 → PWM duty = 35/104 ≈ 33% → High 35 µs, rồi Low 69 µs (bit 0).
 */

void encode_bit(uint8_t bit)
{
    if (dma_len >= 256) return;  
    
    if (bit) {
        dma_buf[dma_len++] = 0;       
    } else {
        dma_buf[dma_len++] = T0_HIGH;
    }
}

// --- Encode 1 byte với Start/Stop/Parity bit ---
void encode_byte(uint8_t b)
{
    uint8_t parity_count = 0;
    
    encode_bit(0);
    
    // Data bits (MSB first)
    for (int i = 7; i >= 0; i--) {
        uint8_t bit = (b >> i) & 0x01;
        encode_bit(bit);
        if (bit) parity_count++;  // đếm bit 1 cho parity
    }
    
    // Parity bit (even parity)
    encode_bit(parity_count % 2);
    
    // Stop bit (luôn là 1)
    encode_bit(1);
}

void prepare_frame(uint8_t *data, int len)
{
    dma_len = 0;
    for (int i = 0; i < len; i++) {
        encode_byte(data[i]);
    }
}

void send_knx_frame(uint8_t *data, int len)
{    
  prepare_frame(data, len);
    
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)dma_buf, dma_len);
}

void start_knx_decode(void)
{
	rx_bitcount = 0;
    rx_byte = 0;
    rx_index = 0;
    frame_ready = 0;
    half_seen = 0;
    waiting_for_bit = 0;
    last_capture = 0;
    last_bit_time = 0;
    frame_timeout = 0;
    knx_state = KNX_IDLE;  
    data_bits = 0;
    parity_bit = 0;
    parity_count = 0;
    rx_half_seen = 0;  // Reset half_seen flag
    
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
}

static void knx_handle_bit(uint8_t bit)
{
	switch (knx_state) {
		case KNX_IDLE:
			if (bit == 0) {
				knx_state = KNX_START;
			}
			break;
		case KNX_START:
			if (bit == 0) {
				knx_state = KNX_DATA;
				rx_byte = 0;
				data_bits = 0;
				parity_count = 0;
			} else {
				knx_state = KNX_IDLE; // error
			}
			break;
		case KNX_DATA:
			rx_byte = (rx_byte << 1) | bit;
			if (bit) parity_count++;
				data_bits++;
			if (data_bits == 8) {
				knx_state = KNX_PARITY;
			}
			break;
		case KNX_PARITY:
			parity_bit = bit;
			knx_state = KNX_STOP;
			break;
		case KNX_STOP:
			if (bit == 1) {
				if (((parity_count % 2) == parity_bit) && (rx_index < 31)) {  // Giảm từ 32 xuống 31 để tránh overflow
					rx_frame[rx_index++] = rx_byte;
					frame_ready = 1;
				} else {
					// Parity error hoặc buffer full - reset
					rx_index = 0;
				}
			}
			knx_state = KNX_IDLE;
			rx_byte = 0;
			data_bits = 0;
			parity_count = 0;
			break;
	}
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
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
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_IWDG_Refresh(&hiwdg);
  start_knx_decode();
  
  // Encode và decode có thể chạy song song vì dùng 2 kênh độc lập
  // Channel 2: PWM (encode) - gửi tín hiệu ra bus
  // Channel 3: Input Capture (decode) - nhận tín hiệu từ bus
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t data[] = {0x00, 0xFF, 0x55, 0xAA, 0x00, 0xFF, 0x55, 0xAA};
  uint32_t millis = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if(HAL_GetTick() - millis > 1000) {
        millis = HAL_GetTick();
        send_knx_frame(data, sizeof(data));
    }
    if (frame_ready) {
        frame_ready = 0;
        HAL_UART_Transmit(&huart2, (uint8_t*)rx_frame, rx_index, 100);
        rx_index = 0;  // Reset index sau khi gửi
    }
    
    // Xử lý timeout frame
    if (frame_timeout) {
        frame_timeout = 0;
        start_knx_decode(); // Reset decoder khi timeout
    }

    HAL_IWDG_Refresh(&hiwdg);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 103;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 104;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Callback khi DMA encode hoàn thành
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
        
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_10) {
        if (!rx_half_seen) {
            rx_half_seen = 1;
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
            HAL_TIM_Base_Start_IT(&htim2); // start timeout
        } else {
            // có cạnh thứ 2 ~35us => bit 0
            knx_handle_bit(0);
            rx_half_seen = 0;
            HAL_TIM_Base_Stop_IT(&htim2);
        }
    }
}


void TIM2_IRQHandler(void)
{
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
		if(__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
			knx_handle_bit(1);
			rx_half_seen = 0;
			HAL_TIM_Base_Stop_IT(&htim2);
		}
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
