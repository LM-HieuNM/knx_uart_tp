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
DMA_HandleTypeDef hdma_tim1_ch2;

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
    
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
}

void check_frame_timeout(void)
{
    uint32_t now = HAL_GetTick();
    
    if (waiting_for_bit && (now - last_bit_time) > 200) {
        knx_state = KNX_IDLE;  // idle
        data_bits = 0;
        parity_count = 0;
        rx_byte = 0;
        half_seen = 0;
        waiting_for_bit = 0;
        frame_timeout = 1;
    }
    
    // Nếu đang nhận frame và quá 500ms không có bit nào
    if (knx_state != KNX_IDLE && (now - last_bit_time) > 500) {
        knx_state = KNX_IDLE;
        data_bits = 0;
        parity_count = 0;
        rx_byte = 0;
        half_seen = 0;
        waiting_for_bit = 0;
        frame_timeout = 1;
    }
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_DMA_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_IWDG_Refresh(&hiwdg);
  start_knx_decode();
  
  // Encode và decode có thể chạy song song vì dùng 2 kênh độc lập
  // Channel 2: PWM (encode) - gửi tín hiệu ra bus
  // Channel 3: Input Capture (decode) - nhận tín hiệu từ bus
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    check_frame_timeout();
    
    if (frame_ready) {
        frame_ready = 0;
        HAL_UART_Transmit(&huart2, (uint8_t*)rx_frame, rx_index, 100);
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
  TIM_IC_InitTypeDef sConfigIC = {0};
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Ngắt tràn timer - xử lý bit 1 (không có pulse)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1 && waiting_for_bit) {
        // Timer tràn sau 104µs => bit 1
        last_bit_time = HAL_GetTick();
        
        switch (knx_state) {
            case KNX_IDLE:
                knx_state = KNX_IDLE;
                break;
                
            case KNX_START:
                knx_state = KNX_IDLE;
                break;
                
            case KNX_DATA:
                rx_byte = (rx_byte << 1) | 1;  
                data_bits++;
                parity_count++;  
                
                if (data_bits == 8) {
                    knx_state = KNX_PARITY;  
                }
                break;
                
            case KNX_PARITY: 
                parity_bit = 1;
                knx_state = KNX_STOP; 
                break;
                
            case KNX_STOP:
                if ((parity_count % 2) == 0) {
                    if (rx_index < 32) {
                        rx_frame[rx_index++] = rx_byte;
                    }
                    
                    if (rx_index >= 32) {
                        frame_ready = 1;
                        rx_index = 0;
                    }
                }
                knx_state = KNX_IDLE;
                data_bits = 0;
                parity_count = 0;
                rx_byte = 0;
                break;
        }
        
        waiting_for_bit = 0;  // reset flag
    }
}

// Ngắt Input Capture - xử lý bit 0 (có pulse)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		uint32_t delta = capture - last_capture;
		last_capture = capture;

        if (!half_seen) {
            // Thấy cạnh Rising (bắt đầu bit 0)
            half_seen = 1;
            waiting_for_bit = 1;  
            last_bit_time = HAL_GetTick();  
        } else {
            // Thấy cạnh Falling
            if (delta > (T0_HIGH - T0_TOL) && delta < (T0_HIGH + T0_TOL)) {
                // Đây là pulse High ~35 µs => bit 0
                last_bit_time = HAL_GetTick();
                
                switch (knx_state) {
                    case KNX_IDLE: 
                        knx_state = KNX_START;  
                        break;
                        
                    case KNX_START: 
                        knx_state = KNX_DATA;  
                        data_bits = 0;
                        parity_count = 0;
                        rx_byte = 0;
                        break;
                        
                    case KNX_DATA: 
                        rx_byte = (rx_byte << 1);  
                        data_bits++;
                        
                        if (data_bits == 8) {
                            knx_state = KNX_PARITY;  
                        }
                        break;
                        
                    case KNX_PARITY: 
                        parity_bit = 0;
                        knx_state = KNX_STOP;  
                        break;
                        
                    case KNX_STOP: 
                        knx_state = KNX_IDLE;
                        break;
                }
            } else {
                // Pulse không hợp lệ - có thể là noise
                half_seen = 0;
                waiting_for_bit = 0;
                return;
            }
            half_seen = 0;
            waiting_for_bit = 0;  // đã nhận xong bit
        }

		if ((TIM1->CCER & TIM_CCER_CC3P) == 0) {
			// đang rising -> chuyển sang falling
			TIM1->CCER |= TIM_CCER_CC3P;
		} else {
			// đang falling -> chuyển sang rising
			TIM1->CCER &= ~TIM_CCER_CC3P;
		}
    }
}

// Callback khi DMA encode hoàn thành
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
        
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
