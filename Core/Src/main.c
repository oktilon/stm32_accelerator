/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define XM_Tx			0
#define XM_TxRx			1
#define XM_Timeout		10
#define dummy           0xFF
#define NSS_DISABLE()   (GPIOE->ODR |= 0x0008)
#define NSS_ENABLE()    (GPIOE->ODR &= 0xFFF7)
#define MAX_TICK		30000
#define DS1307_ADDR		0xD0
#define DS1307_Timeout	10
#define DS3231_ADDR		0xD0
#define DS3231_Timeout	10
#define RTC_REG_SEC  0x00
#define RTC_REG_MIN  0x01
#define RTC_REG_HOUR 0x02
#define RTC_REG_WEEK 0x03
#define RTC_REG_DAY  0x04
#define RTC_REG_MON  0x05
#define RTC_REG_YEAR 0x06
#define RTC_ALARM    0x0A
#define RTC_24       0x40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t uMin = 0;
uint8_t uSec = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t xmeet(uint8_t data, int read, uint32_t timeout);
uint8_t read_reg(uint8_t addr);
void write_reg(uint8_t addr, uint8_t data);
uint8_t ds1307_read(uint8_t addr);
void ds1307_write(uint8_t addr, uint8_t data);
uint8_t ds3231_read(uint8_t addr);
void ds3231_write(uint8_t addr, uint8_t data);
uint8_t bcd2dec(uint8_t);
uint8_t dec2bcd(uint8_t);
void set_time(int y, int m, int d, int h, int min, int s);
void set_alarm(int h, int m);
int8_t check_alarm();
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
  uint8_t chipId = 0;
  uint16_t led3cnt = 0;
  uint16_t led4cnt = 0;
  uint16_t led5cnt = 0;
  uint16_t led6cnt = 0;
  uint16_t led3tm = 0;
  uint16_t led4tm = 0;
  uint16_t led5tm = 0;
  uint16_t led6tm = 0;
//  uint8_t accX = 0;
//  uint8_t accY = 0;
//  uint8_t delim = MAX_TICK / 127;

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
  MX_I2C3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  chipId = read_reg(0x0F); // WHO_AM_I register;
//  write_reg(0x20, 0x47); // PD + Zen + Yen + Xen

  set_time(2022, 9, 1, 9, 0, 0);
  set_alarm(9, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    if(++led3cnt > MAX_TICK) led3cnt = 0;
	    if(++led4cnt > MAX_TICK) led4cnt = 0;
	    if(++led5cnt > MAX_TICK) led5cnt = 0;
	    if(++led6cnt > MAX_TICK) led6cnt = 0;

	    if(check_alarm()) {
	    	led3tm = 20000;
	    	led6tm = 20000;
	    }

	    uMin = bcd2dec(ds3231_read(RTC_REG_MIN));
	    uSec = bcd2dec(ds3231_read(RTC_REG_SEC));
	    HAL_Delay(100);

//	    accX = read_reg(0x29);
//	    accY = read_reg(0x2B);
//	    // accZ = read_reg(0x2D);
//
//	    if(accX > 0 && accX < 255) {
//	    	if(accX < 128) {
//	    		led3tm = delim * accX;
//	    		led6tm = 0;
//	    	} else {
//	    		led6tm = delim * (accX - 127);
//	    		led3tm = 0;
//	    	}
//	    } else {
//	    	led6tm = 0;
//	    	led3tm = 0;
//	    }
//
//	    if(accY > 0 && accY < 255) {
//	    	if(accY < 128) {
//	    		led4tm = delim * accY;
//	    		led5tm = 0;
//	    	} else {
//	    		led5tm = delim * (accY - 127);
//	    		led4tm = 0;
//	    	}
//	    } else {
//	    	led4tm = 0;
//	    	led5tm = 0;
//	    }

	    // LD3
	    if(led3cnt == 0) {
	      GPIOD->ODR |= 0x2000;
	    } else if(led3cnt > led3tm) {
	      GPIOD->ODR &= ~0x2000;
	    }
	    // LD4
	    if(led4cnt == 0) {
	      GPIOD->ODR |= 0x1000;
	    } else if(led4cnt > led4tm) {
	      GPIOD->ODR &= ~0x1000;
	    }
	    // LD5
	    if(led5cnt == 0) {
	      GPIOD->ODR |= 0x4000;
	    } else if(led5cnt > led5tm) {
	      GPIOD->ODR &= ~0x4000;
	    }
	    // LD5
	    if(led6cnt == 0) {
	      GPIOD->ODR |= 0x8000;
	    } else if(led6cnt > led6tm) {
	      GPIOD->ODR &= ~0x8000;
	    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return 0;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
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
  /* USER CODE BEGIN I2C3_Init 2 */
  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  SPI1->CR1 |= SPI_CR1_SPE;
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t xmeet(uint8_t data, int read, uint32_t timeout)
{
	uint8_t ret = 0;
	uint32_t tickstart = HAL_GetTick();

	(void)SPI1->DR;
	//!!! Only after send first byte!
	while(!(SPI1->SR & SPI_SR_TXE)) // Tx buffer empty flag
	{
		if((HAL_GetTick() - tickstart) > timeout)
		{
			return ret;
		}
	}
	// Send data
	*((__IO uint8_t *)&(SPI1->DR)) = data;

	if( read > XM_Tx )
	{
		while(!(SPI1->SR & SPI_SR_RXNE)) // Receive buffer Not Empty
		{
			if((HAL_GetTick() - tickstart) > timeout)
			{
				return ret;
			}
		}
		ret = (uint8_t)(SPI1->DR);
	}

	while((SPI1->SR & SPI_SR_BSY)) // Busy flag
	{
		if((HAL_GetTick() - tickstart) > timeout)
		{
			break;
		}
	}

	do {
		__IO uint32_t tmpreg_ovr = 0x00U;
		tmpreg_ovr = SPI1->DR;
		tmpreg_ovr = SPI1->SR;
		(void)tmpreg_ovr;
	} while(0U);

	return ret;
}


uint8_t read_reg(uint8_t addr)
{
	uint8_t data = 0;

	addr |= 0x80; // To read data, SET highest bit of address (LIS302DL Datasheet)

	NSS_ENABLE();
	// HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
	// HAL_SPI_TransmitReceive(&hspi1, &empty, &data, 1, 10);
	xmeet(addr, XM_Tx, XM_Timeout);
	data = xmeet(dummy, XM_TxRx, XM_Timeout);
	NSS_DISABLE();

	return data;
}

void write_reg(uint8_t addr, uint8_t data) {
	NSS_ENABLE();
	// HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
	// HAL_SPI_Transmit(&hspi1, &data, 1, 10);
	xmeet(addr, XM_Tx, XM_Timeout);
	xmeet(data, XM_Tx, XM_Timeout);
	NSS_DISABLE();
}

uint8_t ds1307_read(uint8_t addr) {
	uint8_t data = 0;
	HAL_I2C_Master_Transmit(&hi2c3, DS1307_ADDR, &addr, 1, DS1307_Timeout);
	HAL_I2C_Master_Receive(&hi2c3, DS1307_ADDR, &data, 1, DS1307_Timeout);
	return data;
}
void ds1307_write(uint8_t addr, uint8_t data) {
	uint8_t buf[2] = {addr, data};
	HAL_I2C_Master_Transmit(&hi2c3, DS1307_ADDR, &buf, 2, DS1307_Timeout);
}

void i2c1_start() {
  I2C1->CR1 |= I2C_CR1_ACK; // (1<<10);  // Enable the ACK
  I2C1->CR1 |= I2C_CR1_START; // (1<<8);  // Generate START
}

void i2c1_address(uint8_t addr7bit, uint8_t read) {
  uint8_t addr = addr7bit << 1;
  if(read) {
    addr |= 0x1;
  }
  I2C1->DR = addr;  //  send the address
  while (!(I2C1->SR1 & I2C_SR1_ADDR));  // wait for ADDR bit to set (1<<1)
  uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
}

uint8_t ds3231_read(uint8_t addr) {
	uint8_t data = 0;
	HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDR, &addr, 1, DS3231_Timeout);
	HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDR, &data, 1, DS3231_Timeout);
	return data;
}
void ds3231_write(uint8_t addr, uint8_t data) {
	uint8_t buf[2] = {addr, data};
	HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDR, buf, 2, DS3231_Timeout);
}
uint8_t bcd2dec(uint8_t bcd) {
	uint8_t u = bcd & 0xF;
	uint8_t d = (bcd & 0xF0) >> 4;
	return u + 10 * d;
}
uint8_t dec2bcd(uint8_t dec) {
	uint8_t d = (dec / 10) << 4;
	uint8_t u = dec % 10;
	return d + u;
}
void set_time(int y, int m, int d, int h, int min, int s) {
	ds3231_write(RTC_REG_YEAR, dec2bcd(y % 10));
	ds3231_write(RTC_REG_MON , dec2bcd(m));
	ds3231_write(RTC_REG_DAY , dec2bcd(d));
	ds3231_write(RTC_REG_HOUR, dec2bcd(h));
	ds3231_write(RTC_REG_MIN , dec2bcd(min));
	ds3231_write(RTC_REG_SEC , dec2bcd(s));
}
void set_alarm(int h, int m) {
	ds3231_write(RTC_ALARM + RTC_REG_MIN , dec2bcd(m));
	ds3231_write(RTC_ALARM + RTC_REG_HOUR, dec2bcd(h));
	ds3231_write(RTC_ALARM + RTC_REG_DAY , 0x80);
}
int8_t check_alarm() {
	uint8_t status = ds3231_read(0x0F);
	return (status & 0x2) ? 1 : 0;
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
