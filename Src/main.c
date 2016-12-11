/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "pdm_filter.h"
#define LED_BLUE_ON   GPIOD->BSRRL = GPIO_Pin_15;
#define LED_BLUE_OFF  GPIOD->BSRRH = GPIO_Pin_15;

#define NOTEFREQUENCY 0.015		//frequency of saw wave: f0 = 0.5 * NOTEFREQUENCY * 48000 (=sample rate)
#define NOTEAMPLITUDE 500.0		//amplitude of the saw wave

#define DAC_I2C_ADDR 0x94
#define FS 16000
#define DMA_MAX_SZE                     0xFFFF
#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)

#define PI 3.14159265f

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static PDMFilter_InitStruct pdm_filter;

uint16_t wav[7][256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
static void CS43I22_Init(void);
static void MP45DT02_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t regValue = 0xFF;
uint8_t cmd[10];
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	int i = 0, j = 0;
  int channel = 0;
	int times = 0;
  int C_freq = (int) (48000.0 / 261.63);
  int D_freq = (int) (48000.0 / 293.66);
  int E_freq = (int) (48000.0 / 329.63);
  int F_freq = (int) (48000.0 / 349.23);
  int G_freq = (int) (48000.0 / 392.0);
  int A_freq = (int) (48000.0 / 440.0);
  int B_freq = (int) (48000.0 / 493.88);
  int freq[] = { C_freq, D_freq, E_freq, F_freq, G_freq, A_freq, B_freq }; 
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
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	CS43I22_Init();
  MP45DT02_Init();

  for (j = 0; j < 7; j++) {
    for (i = 0; i < freq[j]; i++) {
      wav[j][i] = sin(2 * PI / freq[j] * i) * 32767;
      // FINDOUT 2 channel?
			// iAudioData[  iDataPos * 2]      = (sin( ( (2 * 3.14159 /* PI */) / 32.0f) * iDataPos ) * 32767) ;
			// iAudioData[ (iDataPos * 2) + 1] = iAudioData[iDataPos * 2];
    }
  }

	HAL_I2S_Transmit_DMA(&hi2s3, wav, C_freq);

  i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    //if (HAL_I2S_GetState(&hi2s3) == HAL_I2S_STATE_READY) {
		//  HAL_I2S_Transmit_DMA(&hi2s3, wav, C_freq);
    //}
		/*
    while( (SPI3->SR & (1 << 1)) == 0);

		SPI3->DR = wav[0][i % freq[0]];
		i++;
    */
		
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 258;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
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
  hi2c1.Init.OwnAddress1 = 0;
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

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_MSB;
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
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
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

  /*Configure GPIO pin : Audio_RST_Pin */
  GPIO_InitStruct.Pin = Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Audio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void CS43I22_Init(void) {
  
  // 4.9 Recommended Power-Up Sequence (p. 31)

  // 1. Hold RESET low until the power supplies are stable.
  // HAL_Delay(100);

  // 2. Bring RESET high
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

  // 3. The default state of the “Power Ctl. 1” register (0x02) is 0x01. Load the desired register settings while
	// keeping the “Power Ctl 1” register set to 0x01.
	cmd[0] = 0x02;
	cmd[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);

	// Load My Setting
	// 7.2 Power Control 1 (p.37)
	// cmd[0] = 0x02;
	// cmd[1] = 0x9E;
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.3 Power Control 2 (p.38)
	cmd[0] = 0x04;
	// cmd[1] = 0xAA; /* 1010 1010 ( Headphone channel is always ON. Speaker channel is always ON. )*/
  cmd[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always on)
  cmd[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
  cmd[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
  cmd[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.4 Clocking Control (p. 38)
	cmd[0] = 0x05;
  // cmd[1] = 0x81; /* 1000 0001 ( Auto-Detect, Double-Speed Mode (DSM - 50 kHz -100 kHz Fs), MCLK Divide By 2 ) */
	cmd[1] = 0x81;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.5 Interface Control 1 (p. 40)
	cmd[0] = 0x06;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 1, 100);
  HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &cmd[1], 1, 100);
	// cmd[0] = 0x06;
	// cmd[1] = 0x07; /* 0000 0111 ( Slave Mode, Not Inverted SCLK, Disabled DSP, I²S up to 24-bit data, 16-bits Audio Wave Length ) */
	cmd[1] &= (1 << 5); // Clear all bits except bit 5 which is reserved
  cmd[1] &= ~(1 << 7);  // Slave
  cmd[1] &= ~(1 << 6);  // Clock polarity: Not inverted
  cmd[1] &= ~(1 << 4);  // No DSP mode
  cmd[1] &= ~(1 << 2);  // Left justified, up to 24 bit (default)
  cmd[1] |=  (3 << 0);  // 16-bit audio word length for I2S interface
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.6 Interface Control 2 (p. 41)
	// cmd[0] = 0x07;
	// cmd[1] = 0x07;
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.7 Passthrough x Select: PassA (p. 42)
	cmd[0] = 0x08;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &cmd[1], 1, 100);
  cmd[1] &= 0xF0;      // Bits [5-7] are reserved
  cmd[1] |=  (1 << 0); // Use AIN1A as source for passthrough
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.7 Passthrough x Select: PassB (p. 42)
	cmd[0] = 0x09;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 1, 100);
  HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &cmd[1], 1, 100);
  cmd[1] &= 0xF0;      // Bits [5-7] are reserved
  cmd[1] |=  (1 << 0); // Use AIN1B as source for passthrough
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.10 Playback Control 1 (p. 43)
	// cmd[0] = 0x0D;
	// cmd[1] = 0x70; /* 0111 0000 ( 0.6047 Headphone Analog Gain, Playback Volume Setting B=A )*/
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.8 Analog ZC and SR Settings (Address 0Ah) (p. 42)
	// cmd[0] = 0x0A;
	// cmd[1] = 0x00; /* xxxx 0000 */
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
	// 7.11 Miscellaneous Controls (Address 0Eh) (p. 44)
	cmd[0] = 0x0E;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &cmd[1], 1, 100);
  cmd[1] &= ~(1 << 7);   // Disable passthrough for AIN-A
  cmd[1] &= ~(1 << 6);   // Disable passthrough for AIN-B
  cmd[1] |=  (1 << 5);   // Mute passthrough on AIN-A
  cmd[1] |=  (1 << 4);   // Mute passthrough on AIN-B
  cmd[1] &= ~(1 << 3);   // Changed settings take affect immediately
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 100);
  // 7.12 Playback Control 2 (Address 0Fh) (p. 45)
  cmd[0] = 0x0F;
  cmd[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 100);
  // 7.23 Limiter Control 1, Min/Max Thresholds  (p. 53)
	// cmd[0] = 0x27;
	// cmd[1] = 0x00; /* 0000 0000 ( Limiter Maximum Threshold 0 dB, Limiter Cushion Threshold 0 dB ) */
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.14 PCMx Volume: PCMA (p. 47)
	cmd[0] = 0x1A;
	cmd[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 3, 1000);
  // 7.14 PCMx Volume: PCMB (p. 47)
  cmd[0] = 0x1B; // | 0x80;
  cmd[1] = 0x00; // 0000 1010 ( PCM Channel B Volume )
  HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  
	// 7.18 Tone Control (p. 50)
	// cmd[0] = 0x1F;
	// cmd[1] = 0x0F; // 0000 1111 ( Treble Gain +12.0 dB, Bass Gain -10.5 dB )
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.19 Master Volume Control: MSTA (p. 51)
	// cmd[0] = 0x20;
	// cmd[1] = 0x18; // 0001 1000
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.19 Master Volume Control: MSTB (p. 52)
	// cmd[0] = 0x21;
	// cmd[1] = 0x18; // 0001 1000
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.20 Headphone Volume Control: HPA (p. 51)
	// cmd[0] = 0x22;
	// cmd[1] = 0x00; // 0000 0000 ( Headphone Volume Control 0 dB )
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.20 Headphone Volume Control: HPB (p. 51)
	// cmd[0] = 0x23;
	// cmd[1] = 0x00; // 0000 0000 ( Headphone Volume Control 0 dB )
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.15 Beep Frequency & On Time (p. 48)
	// cmd[0] = 0x1C;
	// cmd[1] = 0x58; // 0101 1000 ( Beep Frequency 774.19 Hz G5, Beep On Time ~2.80 s )
	// HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.16 Beep Volume & Off Time (p. 48)
	// cmd[0] = 0x1D;
	// cmd[1] = 0x00; // 0000 0000 ( Beep Off Time ~1.23 s, Beep Volume -56 dB )
	//vHAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 7.17 Beep & Tone Configuration (p. 49)
	//vcmd[0] = 0x1E;
	//vcmd[1] = 0x60; // 0110 0000 ( Single Beep, Beep Mix Disable )
	//vHAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
	
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
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &cmd[1],1,1000);
	cmd[1] | 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 2, 1000);
  // 4.4. Write ‘0’b to bit 7 in register 0x32.
	cmd[0] = 0x32;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, cmd, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &cmd[1],1,1000);
	cmd[1] &= ~(0x80);
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

  // DEBUG: ZONE keep breakpoint here
  /*
	uint8_t iReg_02 = 0x02;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, &iReg_02, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &iReg_02, 1, 100);
	uint8_t iReg_04 = 0x04;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, &iReg_04, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &iReg_04, 1, 100);
	uint8_t iReg_05 = 0x05;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, &iReg_05, 1, 100);
	HAL_I2C_Master_Receive( &hi2c1, DAC_I2C_ADDR, &iReg_05, 1, 100);
	uint8_t iReg_06 = 0x06;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, &iReg_06, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &iReg_06, 1, 100);
	uint8_t iReg_0E = 0x0E;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, &iReg_0E, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &iReg_0E, 1, 100);
	uint8_t iReg_0F = 0x0F;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, &iReg_0F, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &iReg_0F, 1, 100);
	uint8_t iReg_1A = 0x1A;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, &iReg_1A, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &iReg_1A, 1, 100);
	uint8_t iReg_1B = 0x1B;
	HAL_I2C_Master_Transmit(&hi2c1, DAC_I2C_ADDR, &iReg_1B, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, DAC_I2C_ADDR, &iReg_1B, 1, 100);
   */
}

static void MP45DT02_Init(void) {
  pdm_filter.LP_HZ = 8000;
  pdm_filter.HP_HZ = 10;
  pdm_filter.Fs = FS;
  pdm_filter.Out_MicChannels = 1;
  pdm_filter.In_MicChannels = 1;
  
  PDM_Filter_Init(&pdm_filter);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s3)
{
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s3)
{
	if(hi2s3->Instance==I2S3) {
		HAL_I2S_Transmit_DMA(&hi2s3, wav[0], C_freq);
	}
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
