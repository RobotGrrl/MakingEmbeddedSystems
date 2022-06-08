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
#include "adc.h"
#include "dcmi.h"
#include "dfsdm.h"
#include "i2c.h"
#include "usart.h"
#include "quadspi.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "../../../Drivers/BSP/STM32L496G-Discovery/stm32l496g_discovery.h"
#include "../../../Drivers/BSP/STM32L496G-Discovery/stm32l496g_discovery_lcd.h"
#include "../../../Drivers/BSP/STM32L496G-Discovery/stm32l496g_discovery_ts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_TOGGLE_DELAY         100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t xpos = 0;
uint8_t ypos = 120;
long last_refresh = 0;
Point prev;
Point new;
TS_StateTypeDef ts_result;
bool flip = false;
bool circle_selected = false;
uint32_t last_ts;
bool dimmed_screen = false;

bool led_on = false;
static uint32_t TimingDelay;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemPower_Config(void);
void SystemClock_Decrease(void);
void prepareForSleep(void);
void enterSleep(void);
void awakeFromSleep(void);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_DCMI_Init();
  MX_DFSDM1_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* LCD Init */
	if (BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE) == LCD_ERROR)
	{
		/* Initialization Error */
		Error_Handler();
	}

	// the lcd bsp includes the fontNN.c files in the Utilities directory
	// in that file, a struct is declared: FontNN
	BSP_LCD_SetFont(&Font24);

	// the display is 240 px tall, XYZ px wide
	BSP_LCD_DisplayStringAt(0, 240 - 65, (uint8_t *)"Beep", CENTER_MODE);

	HAL_Delay(5000);

	// LED
	BSP_LED_Init(LED1_PIN);
	BSP_LED_Init(LED2_PIN);
	BSP_LED_Off( LED1_PIN ); // orange labeled LD1
	BSP_LED_Off( LED2_PIN ); // doesn't work


	BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  	// by changing the text, we can prove that it's resetting when pressing
  	// the reset button - because the display will say Beep instead of the
  	// other two possibilities
  	if(TimingDelay == 0) {
  		/* Toggle LED1 */
			if(led_on) {
				BSP_LED_Off(LED1);
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				BSP_LCD_DisplayStringAt(0, 240 - 65, (uint8_t *)"Zweep", CENTER_MODE);
			} else {
				BSP_LED_On(LED1);
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				BSP_LCD_DisplayStringAt(0, 240 - 65, (uint8_t *)"Fleep", CENTER_MODE);
			}
			led_on = !led_on;
  	}


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
  RCC_OscInitStruct.PLL.PLLN = 24;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK
                              |RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void prepareForSleep(void) {

	/* Configure the system Power */
	SystemPower_Config();

	/* Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* User push-button (lines 10 to 15) will be used to wakeup the system from STOP mode */
	// idk what to do for this
	// added interrupt for joy sel & joy down
	//BSP_PB_Init(JOY_SEL, JOY_MODE_EXTI);

	/* Disable Prefetch Buffer */
	__HAL_FLASH_PREFETCH_BUFFER_DISABLE();

	/* Enable Flash power down mode during Sleep mode     */
	/* (uncomment this line if power consumption figures  */
	/*  must be measured with Flash still on in Low Power */
	/*  Sleep mode)                                       */
	__HAL_FLASH_SLEEP_POWERDOWN_ENABLE();

	/* Reset all RCC Sleep and Stop modes register to */
	/* improve power consumption                      */
	RCC->AHB1SMENR  = 0x0;
	RCC->AHB2SMENR  = 0x0;
	RCC->AHB3SMENR  = 0x0;

	RCC->APB1SMENR1 = 0x0;
	RCC->APB1SMENR2 = 0x0;
	RCC->APB2SMENR  = 0x0;

}

void enterSleep(void) {

	/* Insert 5 seconds delay */
	HAL_Delay(5000);

	/* Reduce the System clock to below 2 MHz */
	SystemClock_Decrease();

	/* Suspend Tick increment to prevent wakeup by Systick interrupt.         */
	/* Otherwise the Systick interrupt will wake up the device within 1ms     */
	/* (HAL time base).                                                       */
	HAL_SuspendTick();

	/* De-init LED1 */
	/*BSP_LED_DeInit(LED1);*/

	/* Enter Sleep Mode, wake up is done once User push-button is pressed */
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	/* ... Low-power SLEEP mode ... */

}

void awakeFromSleep(void) {
	BSP_LED_On(LED1);

	/* System is Low Power Run mode when exiting Low Power Sleep mode,
		 disable low power run mode and reset the clock to initialization configuration */
	HAL_PWREx_DisableLowPowerRunMode();

	/* Configure the system clock for the RUN mode */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	// should this go here too? added it
	PeriphCommonClock_Config();

	/* Re-init LED1 to toggle during Run mode */
	/*BSP_LED_Init(LED1);*/

	/* Resume Tick interrupt if disabled prior to Low Power Sleep mode entry */
	HAL_ResumeTick();
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == JOY_DOWN_Pin) {
		//prepareForSleep();
		//enterSleep();

		// simpler version
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	}

	if(GPIO_Pin == JOY_RIGHT_Pin) {
		//awakeFromSleep();

		// simpler version
		HAL_ResumeTick();
	}

}

/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
  *            + System Running at MSI (~100 KHz)
  *            + Flash 0 wait state
  *            + Code running from Internal FLASH
  *            + Wakeup using EXTI Line (User push-button PC.13)
  * @param  None
  * @retval None
  */
void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

      /* Set all GPIO in analog state to reduce power consumption */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = GPIO_PIN_All;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOI, &GPIO_InitStructure);

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_GPIOF_CLK_DISABLE();
    __HAL_RCC_GPIOG_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
    __HAL_RCC_GPIOI_CLK_DISABLE();

}

/**
  * @brief  System Clock Speed decrease
  *         The system Clock source is shifted from HSI to MSI
  *         while at the same time, MSI range is set to RCC_MSIRANGE_0
  *         to go down to 100 KHz
  * @param  None
  * @retval None
  */
void SystemClock_Decrease(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Disable HSI to reduce power consumption since MSI is used from that point */
  __HAL_RCC_HSI_DISABLE();

}

/**
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  //HAL_IncTick(); // it's already incremented in _it.c
  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
  else
  {
    // don't do this, it results in a usagefault
  	/* Toggle LED1 */
    //BSP_LED_Toggle(LED1); // results in a usagefault
//  	if(led_on) {
//  		BSP_LED_Off(LED1);
//  	} else {
//  		BSP_LED_On(LED1);
//  	}
//  	led_on = !led_on;
    TimingDelay = LED_TOGGLE_DELAY;
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

  // from the pwr example - unclear why this is
  /* Suspend tick */
  //HAL_SuspendTick();

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
