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
#include "adc.h"
#include "cordic.h"
#include "crc.h"
#include "dma.h"
#include "fmac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "pan_tompkins.h"
#include <string.h>

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

#define ADC_DMA_LENGTH       32u
#define SAMPLE_QUEUE_LENGTH  128u
#define PACKET_QUEUE_LENGTH  64u
#define ECG_PACKET_LENGTH    24u

static uint16_t adc_dma_buffer[ADC_DMA_LENGTH];
static uint16_t sample_queue[SAMPLE_QUEUE_LENGTH];
static uint8_t packet_queue[PACKET_QUEUE_LENGTH][ECG_PACKET_LENGTH];
static uint8_t uart_tx_packet[ECG_PACKET_LENGTH];
static volatile uint16_t sample_head;
static volatile uint16_t sample_tail;
static uint8_t packet_head;
static uint8_t packet_tail;
static volatile bool uart_tx_busy;
static volatile uint32_t sample_overruns;
static PT_Handle pan_tompkins;
static uint32_t last_heartbeat_tick;

static void queue_samples(const uint16_t *samples, uint32_t count)
{
  for (uint32_t i = 0u; i < count; i++)
  {
    uint16_t next = (uint16_t)((sample_head + 1u) % SAMPLE_QUEUE_LENGTH);
    if (next == sample_tail)
    {
      sample_overruns++;
      return;
    }
    sample_queue[sample_head] = samples[i];
    sample_head = next;
  }
}

static bool dequeue_sample(uint16_t *sample)
{
  if (sample_tail == sample_head)
    return false;

  *sample = sample_queue[sample_tail];
  sample_tail = (uint16_t)((sample_tail + 1u) % SAMPLE_QUEUE_LENGTH);
  return true;
}

static uint8_t checksum_packet(const uint8_t *packet)
{
  uint8_t checksum = 0u;
  for (uint32_t i = 2u; i < 22u; i++)
    checksum ^= packet[i];
  return checksum;
}

static void make_packet(uint8_t *packet, uint16_t raw_adc)
{
  uint8_t qrs = PT_QRSDetected(&pan_tompkins) ? 1u : 0u;
  uint8_t lead_off = (HAL_GPIO_ReadPin(LO_PLUS_GPIO_Port, LO_PLUS_Pin)
                   == GPIO_PIN_SET)
                   || (HAL_GPIO_ReadPin(LO_MINUS_GPIO_Port, LO_MINUS_Pin)
                   == GPIO_PIN_SET);

  memset(packet, 0, ECG_PACKET_LENGTH);
  packet[0] = 0xAAu;
  packet[1] = 0x55u;
  memcpy(&packet[2], &raw_adc, sizeof(raw_adc));
  memcpy(&packet[4], &pan_tompkins.last_bpf, sizeof(float));
  memcpy(&packet[8], &pan_tompkins.last_mwi, sizeof(float));
  memcpy(&packet[12], &pan_tompkins.heart_rate_bpm, sizeof(float));
  packet[16] = qrs;
  packet[17] = lead_off ? 1u : 0u;
  memcpy(&packet[18], &pan_tompkins.sample_index, sizeof(uint32_t));
  packet[22] = checksum_packet(packet);
  packet[23] = 0xFFu;
}

static void queue_packet(const uint8_t *packet)
{
  uint8_t next = (uint8_t)((packet_head + 1u) % PACKET_QUEUE_LENGTH);
  if (next == packet_tail)
    return;

  memcpy(packet_queue[packet_head], packet, ECG_PACKET_LENGTH);
  packet_head = next;
}

static void service_uart(void)
{
  if (uart_tx_busy || packet_tail == packet_head)
    return;

  memcpy(uart_tx_packet, packet_queue[packet_tail], ECG_PACKET_LENGTH);
  packet_tail = (uint8_t)((packet_tail + 1u) % PACKET_QUEUE_LENGTH);
  uart_tx_busy = true;
  if (HAL_UART_Transmit_IT(&huart2, uart_tx_packet, ECG_PACKET_LENGTH)
      != HAL_OK)
  {
    uart_tx_busy = false;
  }
}

static void process_samples(void)
{
  uint16_t raw_adc;
  uint8_t packet[ECG_PACKET_LENGTH];

  while (dequeue_sample(&raw_adc))
  {
    PT_Update(&pan_tompkins, raw_adc);
    make_packet(packet, raw_adc);
    queue_packet(packet);
  }
}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_CRC_Init();
  MX_FMAC_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  static const uint8_t startup_message[] = "STM32G431 clock=170MHz USART2=115200\r\n";
  if (HAL_UART_Transmit(&huart2, (uint8_t *)startup_message,
                        sizeof(startup_message) - 1u, 100u) != HAL_OK)
  {
    Error_Handler();
  }

  PT_Init(&pan_tompkins);
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer,
                        ADC_DMA_LENGTH) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    process_samples();
    service_uart();

    if ((HAL_GetTick() - last_heartbeat_tick) >= 500u)
    {
      last_heartbeat_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
    queue_samples(adc_dma_buffer, ADC_DMA_LENGTH / 2u);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
    queue_samples(&adc_dma_buffer[ADC_DMA_LENGTH / 2u],
                  ADC_DMA_LENGTH / 2u);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
    uart_tx_busy = false;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
