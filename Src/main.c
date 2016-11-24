/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
//Defines
#define LED_BLUE_ON   GPIOD->BSRRL = GPIO_Pin_15;
#define LED_BLUE_OFF  GPIOD->BSRRH = GPIO_Pin_15;

#define NOTEFREQUENCY 0.015		//frequency of saw wave: f0 = 0.5 * NOTEFREQUENCY * 48000 (=sample rate)
#define NOTEAMPLITUDE 500.0		//amplitude of the saw wave

#define DAC_I2C_ADDR 0x94

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct {
  float tabs[8];
  float params[8];
  uint8_t currIndex;
} fir_8;

uint32_t sampleCounter = 0;
int16_t sample = 0xE5;

float sawWave = 0.0;

float filteredSaw = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void CS43l22_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t regValue = 0xFF;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t cmd[10];

  /* Initialize I2C*/
  char data;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  CS43I22_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
    uint16_t data_l = 0x8080;
    HAL_I2S_Transmit(&hi2s3, &data, 1, 1000);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 102;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PB10   ------> I2S2_CK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           PD4 PD6 */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void CS43I22_Init(void) {
  
  // 4.9 Recommended Power-Up Sequence (p. 31)

  // 1. Hold RESET low until the power supplies are stable.
  HAL_Delay(100);

  // 2. Bring RESET high
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

  // 3. The default state of the “Power Ctl. 1” register (0x02) is 0x01. Load the desired register settings while
  // keeping the “Power Ctl 1” register set to 0x01.
  cmd[0] = 0x02;
  cmd[1] = 0x01;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // Load My Setting
  // 7.3 Power Control 2 (p.38)
  cmd[0] = 0x04;
  cmd[1] = 0xAA; /* 1010 1010 ( Headphone channel is always ON. Speaker channel is always ON. )*/
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.4 Clocking Control (p. 38)
  cmd[0] = 0x05;
  cmd[1] = 0x81; /* 1000 0001 ( Auto-Detect, Double-Speed Mode (DSM - 50 kHz -100 kHz Fs), MCLK Divide By 2 ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.5 Interface Control 1 (p. 40)
  cmd[0] = 0x06;
  cmd[1] = 0x07; /* 0000 0111 ( Slave Mode, Not Inverted SCLK, Disabled DSP, I²S up to 24-bit data, 16-bits Audio Wave Length ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.6 Interface Control 2 (p. 41)
  // cmd[0] = 0x07;
  // cmd[1] = 0x07;
  // HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.7 Passthrough x Select: PassA (p. 42)
  cmd[0] = 0x08;
  cmd[1] = 0x07 /* xxxx 0111 */;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.7 Passthrough x Select: PassB (p. 42)
  cmd[0] = 0x09;
  cmd[1] = 0x07; /* xxxx 0111 */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.8 Analog ZC and SR Settings (Address 0Ah) (p. 42)
  cmd[0] = 0x0A;
  cmd[1] = 0x00; /* xxxx 0000 */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.10 Playback Control 1 (p. 43)
  cmd[0] = 0x0D;
  cmd[1] = 0x70; /* 0111 0000 ( 0.6047 Headphone Analog Gain, Playback Volume Setting B=A )*/
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.23 Limiter Control 1, Min/Max Thresholds  (p. 53)
  cmd[0] = 0x27;
  cmd[1] = 0x00; /* 0000 0000 ( Limiter Maximum Threshold 0 dB, Limiter Cushion Threshold 0 dB ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.14 PCMx Volume: PCMA (p. 47)
  cmd[0] = 0x1A; // | 0x80;
  cmd[1] = 0x0A; /* 0000 1010 ( PCM Channel A Volume ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.14 PCMx Volume: PCMB (p. 47)
  cmd[0] = 0x1A; // | 0x80;
  cmd[1] = 0x0A; /* 0000 1010 ( PCM Channel B Volume ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.18 Tone Control (p. 50)
  cmd[0] = 0x1F;
  cmd[1] = 0x0F; /* 0000 1111 ( Treble Gain +12.0 dB, Bass Gain -10.5 dB ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.19 Master Volume Control: MSTA (p. 51)
  cmd[0] = 0x20;
  cmd[1] = 0x18; /* 0001 1000 */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.19 Master Volume Control: MSTB (p. 52)
  cmd[0] = 0x21;
  cmd[1] = 0x18; /* 0001 1000 */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.20 Headphone Volume Control: HPA (p. 51)
  cmd[0] = 0x22;
  cmd[1] = 0x00; /* 0000 0000 ( Headphone Volume Control 0 dB )*/
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.20 Headphone Volume Control: HPB (p. 51)
  cmd[0] = 0x23;
  cmd[1] = 0x00; /* 0000 0000 ( Headphone Volume Control 0 dB )*/
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.15 Beep Frequency & On Time (p. 48)
  cmd[0] = 0x1C;
  cmd[1] = 0x58; /* 0101 1000 ( Beep Frequency 774.19 Hz G5, Beep On Time ~2.80 s ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.16 Beep Volume & Off Time (p. 48)
  cmd[0] = 0x1D;
  cmd[1] = 0x00; /* 0000 0000 ( Beep Off Time ~1.23 s, Beep Volume -6 dB ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.17 Beep & Tone Configuration (p. 49)
  cmd[0] = 0x1E;
  cmd[1] = 0xE0; /* 1110 0000 ( Continuous Beep, Beep Mix Disable ) */
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 4. Load the required initialization settings listed in Section 4.11
  // 4.1. Write 0x99 to register 0x00.
  cmd[0] = 0x00;
  cmd[1] = 0x99;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 4.2. Write 0x80 to register 0x47
  cmd[0] = 0x47;
  cmd[1] = 0x80;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 4.3. Write ‘1’b to bit 7 in register 0x32.
  cmd[0] = 0x32;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 1, 1000);
  HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &regValue,1,1000);
  cmd[0] = 0x32;
  cmd[1] = regValue | 0x80;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 4.4. Write ‘0’b to bit 7 in register 0x32.
  cmd[0] = 0x32;
  cmd[1] = regValue & (~0x80);
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 4.5. Write 0x00 to register 0x00.
  cmd[0] = 0x00;
  cmd[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);

  // 5. Apply MCLK at the appropriate frequency, as discussed in Section 4.6. SCLK may be applied or set to
  // master at any time; LRCK may only be applied or set to master while the PDN bit is set to 1.

  // 6. Set the “Power Ctl 1” register (0x02) to 0x9E
  cmd[0] = 0x02;
  cmd[1] = 0x9E;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);

  // 7. Bring RESET low if the analog or digital supplies drop below the recommended operating condition to
  // prevent power glitch related issues.

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
