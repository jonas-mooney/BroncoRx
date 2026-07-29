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
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#pragma pack(push, 1)
typedef struct {
    uint16_t x;
    uint16_t y;
} joystick_payload_t;
#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define CE_Port GPIOA
#define CE_Pin GPIO_PIN_3

#define CSN_Port GPIOA
#define CSN_Pin GPIO_PIN_4

#define NRF24_CMD_R_REGISTER  0x00
#define NRF24_REG_CONFIG      0x00

#define NRF24_CMD_W_REGISTER    0x20
#define NRF24_CMD_W_TX_PAYLOAD  0xA0
#define NRF24_CMD_FLUSH_TX      0xE1

#define NRF24_REG_EN_AA         0x01
#define NRF24_REG_RF_CH         0x05
#define NRF24_REG_STATUS        0x07
#define NRF24_REG_RX_ADDR_P0    0x0A
#define NRF24_REG_TX_ADDR       0x10

// Rx
#define NRF24_CMD_R_RX_PAYLOAD  0x61
#define NRF24_CMD_FLUSH_RX      0xE2

#define NRF24_REG_RX_PW_P0       0x11

#define NRF24_REG_FIFO_STATUS 0x17

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t nrf24_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { NRF24_CMD_R_REGISTER | reg, 0xFF };
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);

    return rx[1];
}

void nrf24_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { NRF24_CMD_W_REGISTER | reg, value };
    uint8_t rx[2];

    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);
}

void nrf24_write_addr_reg(uint8_t reg, const uint8_t *addr, uint8_t len)
{
    uint8_t cmd = NRF24_CMD_W_REGISTER | reg;

    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)addr, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);
}

void nrf24_write_payload(const uint8_t *data, uint8_t len)
{
    uint8_t cmd = NRF24_CMD_W_TX_PAYLOAD;

    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);
}

void nrf24_pulse_ce(void)
{
    HAL_GPIO_WritePin(CE_Port, CE_Pin, GPIO_PIN_SET);
    HAL_Delay(1);   // comfortably clears the 10µs minimum (Thce)
    HAL_GPIO_WritePin(CE_Port, CE_Pin, GPIO_PIN_RESET);
}

// Rx
void nrf24_read_payload(uint8_t *data, uint8_t len)
{
    uint8_t cmd = NRF24_CMD_R_RX_PAYLOAD;
    uint8_t dummy[32] = {0};   // scratch buffer, only need `len` bytes back

    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi1, dummy, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);
}

void nrf24_flush_rx(void)
{
    uint8_t cmd = NRF24_CMD_FLUSH_RX;

    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  printf("User code begin 2\r\n");

      // set PWR_UP = 1
  // set PRIM_RX = 0
  // wait for the startup time, and configure any other radio registers you nee

  // Begin config for receive
HAL_Delay(100);
uint8_t cfg = nrf24_read_reg(NRF24_REG_CONFIG);
printf("CONFIG initial = 0x%02X\r\n", cfg);

// Probe: write something CONFIG is definitely not already at
nrf24_write_reg(NRF24_REG_CONFIG, 0x00);
uint8_t probe = nrf24_read_reg(NRF24_REG_CONFIG);
printf("CONFIG after 0x00 probe = 0x%02X\r\n", probe);

// Now the real config sequence
uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
nrf24_write_addr_reg(NRF24_REG_RX_ADDR_P0, addr, 5);
nrf24_write_reg(NRF24_REG_RX_PW_P0, 4);
nrf24_write_reg(NRF24_REG_RF_CH, 76);
nrf24_write_reg(NRF24_REG_CONFIG, 0x0B);

// Read everything back to confirm each write actually landed
uint8_t rx_pw = nrf24_read_reg(NRF24_REG_RX_PW_P0);
uint8_t rf_ch = nrf24_read_reg(NRF24_REG_RF_CH);
uint8_t cfg_final = nrf24_read_reg(NRF24_REG_CONFIG);
printf("RX_PW_P0 = 0x%02X, RF_CH = 0x%02X, CONFIG = 0x%02X\r\n", rx_pw, rf_ch, cfg_final);

HAL_Delay(5);
nrf24_flush_rx();
HAL_GPIO_WritePin(CE_Port, CE_Pin, GPIO_PIN_SET);
  // End config for receive

  // One-time verification: confirm RX_ADDR_P0 actually landed
  uint8_t addr_check[5] = {0};
  uint8_t cmd = NRF24_CMD_R_REGISTER | NRF24_REG_RX_ADDR_P0;
  uint8_t dummy[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_TransmitReceive(&hspi1, dummy, addr_check, 5, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);

  printf("RX_ADDR_P0 = %02X %02X %02X %02X %02X\r\n",
         addr_check[0], addr_check[1], addr_check[2], addr_check[3], addr_check[4]);


  // Motor config start
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // Motor config end

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint8_t status = nrf24_read_reg(NRF24_REG_STATUS);
    uint8_t cfg_check = nrf24_read_reg(NRF24_REG_CONFIG);
    uint8_t fifo_status = nrf24_read_reg(NRF24_REG_FIFO_STATUS);
    // printf("STATUS = 0x%02X, CONFIG = 0x%02X, FIFO_STATUS = 0x%02X\r\n", status, cfg_check, fifo_status);

    if (status & 0x40) {   // RX_DR bit
        // uint8_t payload[1]; // commented out when adding joystick data
        // nrf24_read_payload(payload, 1); // commented out when adding joystick data
        // printf("Received: 0x%02X\r\n", payload[0]);  // commented out when adding joystick data

        joystick_payload_t received;
        nrf24_read_payload((uint8_t *)&received, sizeof(received));
        printf("X: %u  Y: %u\r\n", received.x, received.y);

        nrf24_write_reg(NRF24_REG_STATUS, 0x40);  // clear RX_DR
    }

    HAL_Delay(200);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
    HAL_Delay(300);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_Delay(300);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
    HAL_Delay(300);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_Delay(300);

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
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
