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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// add these directories to the build paths in project properties
#include <stdbool.h>
#include "vl53l0x_api.h"
#include "stm32l496g_discovery.h"
#include "stm32l496g_discovery_lcd.h"
#include "stm32l496g_discovery_ts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ERR_DETECT             -1
#define TIMED_RANGING_PERIOD   50    // in ms
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
bool gone_sleep = false;
static uint32_t TimingDelay;

/**
 * Global ranging struct
 */
VL53L0X_RangingMeasurementData_t RangingMeasurementData;

/** leaky factor for filtered range
 *
 * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
 *
 * */
int LeakyFactorFix8 = (int)( 0.6 *256);

/** How many device detect set by @a DetectSensors()*/
int nDevPresent=0;

/** bit is index in VL53L0XDevs that is not necessary the dev id of the BSP */
int nDevMask;
// EK ^ that is probably not needed after removing the for loop completely

VL53L0X_Dev_t VL53L0XDev = {
		.Id=0,
		.DevLetter='e',
		.I2cHandle=&hi2c1,
		.I2cDevAddr=0x29 // EK: documentation says address is 0x29, why did the code say 0x52?
};

uint16_t range;
uint16_t range_mm;

uint32_t range_mm_sum = 0;
uint8_t sum_count = 0;
uint8_t sum_skip = 0;
float range_mm_avg = 0.0;
unsigned long last_sample = 0;

float all_samples[120];
uint16_t num_samples = 30;
uint16_t sample_index = 0;

uint16_t raw;
float raw_in;
float raw_mm;
uint8_t buf[12];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void ResetAndDetectSensor(int SetDisplay); // TODO: add this function
void SystemPower_Config(void);
void SystemClock_Decrease(void);
void prepareForSleep(void);
void enterSleep(void);
void awakeFromSleep(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Handle Error
 *
 * Set err on display and loop forever
 * @param err Error case code
 */
void HandleError(int err){
    char msg[16];
    sprintf(msg,"Er%d", err);
    while(1){};
}

/**
 * Reset all sensor then do presence detection
 *
 * All present devices are data initiated and assigned to their final I2C address
 * @return
 */
int DetectSensors(int SetDisplay) {
    int i;
    uint16_t Id;
    int status;
    int FinalAddress;

    // EK unused char PresentMsg[5]="    ";
    /* Reset all */
    nDevPresent = 0;

    // EK: commenting this out, unclear if it's needed w/o an i2c expander
    /*
    for (i = 0; i < 3; i++)
        status = XNUCLEO53L0A1_ResetId(i, 0);
    */
    // EK: this function is in the HAL for that nucleo board
    // why would it be located there if it is for the sensor?
    // what is that code doing - something about an expander? why?

    /* detect all sensors (even on-board)*/
    // EK no need for for loop for (i = 0; i < 3; i++) {

    i = 0; // EK: in case anything is out there using i

    		VL53L0X_Dev_t *pDev;
        pDev = &VL53L0XDev; // EK: only using one //&VL53L0XDevs[i];
        pDev->I2cDevAddr = 0x29; // EK: documentation says address is 0x29, why did the code say 0x52? //0x52;
        pDev->Present = 0;
        status = 1; // EK: is this needed? XNUCLEO53L0A1_ResetId( pDev->Id, 1);
        HAL_Delay(2); // EK: why is this here
        FinalAddress= 0x29; // EK: see comment a few lines above about address // 0x52+(i+1)*2;

        do {
        	/* Set I2C standard mode (400 KHz) before doing the first register access */
        	// EK: why is this, just do it  if (status == VL53L0X_ERROR_NONE)
        	// EK let's not do this status = VL53L0X_WrByte(pDev, 0x88, 0x00);

        	/* Try to read one register using default 0x52 address */
            status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
            if (status) {
                printf("#%d Read id fail\n", i);
                break;
            }
            if (Id == 0xEEAA) {
				/* Sensor is found => Change its I2C address to final one */
                status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
                if (status != 0) {
                    printf("#%d VL53L0X_SetDeviceAddress fail\n", i);
                    break;
                }
                pDev->I2cDevAddr = FinalAddress;
                /* Check all is OK with the new I2C address and initialize the sensor */
                status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
                if (status != 0) {
					printf("#%d VL53L0X_RdWord fail\n", i);
					break;
				}

                status = VL53L0X_DataInit(pDev);
                if( status == 0 ){
                    pDev->Present = 1;
                }
                else{
                    printf("VL53L0X_DataInit %d fail\n", i);
                    break;
                }
                printf("VL53L0X %d Present and initiated to final 0x%x\n", pDev->Id, pDev->I2cDevAddr);
                nDevPresent++;
                nDevMask |= 1 << i;
                pDev->Present = 1;
            }
            else {
                printf("#%d unknown ID %x\n", i, Id);
                status = 1;
            }
        } while (0);
        /* if fail r can't use for any reason then put the  device back to reset */
        if (status) {
        	  printf("something failed"); // EK
            // EK why is this needed XNUCLEO53L0A1_ResetId(i, 0);
        }

    // EK no need for for loop }

    // EK: not needed
    /* Display detected sensor(s) */
    /*
    if( SetDisplay ){
        for(i=0; i<3; i++){
            if( VL53L0XDevs[i].Present ){
                PresentMsg[i+1]=VL53L0XDevs[i].DevLetter;
            }
        }
        PresentMsg[0]=' ';
        XNUCLEO53L0A1_SetDisplayString(PresentMsg);
        HAL_Delay(1000);
    }
    */

    return nDevPresent;
}

void ResetAndDetectSensor(int SetDisplay){
    int nSensor;
    nSensor = DetectSensors(SetDisplay);

    // EK added this
    if( VL53L0XDev.Present==0 )
    {
    	HandleError(ERR_DETECT);
    }

    /* at least one sensor and if one it must be the built-in one  */
    if( (nSensor <=0) ) { // EK ||  (nSensor ==1 && VL53L0XDevs[1].Present==0) ){
        HandleError(ERR_DETECT);
    }
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if(htim == &htim6) {
		// toggle ARD_D7
		// this should be 25 Hz, every 40 ms
		HAL_GPIO_TogglePin(ARD_D7_GPIO_Port, ARD_D7_Pin);
	}

}

void drawCircle(uint16_t x, uint16_t y) {

	if(!circle_selected) {
		BSP_LCD_SetTextColor( LCD_COLOR_DARKMAGENTA );
		BSP_LCD_FillCircle(x, y, 20);
	} else {
		BSP_LCD_SetTextColor( LCD_COLOR_GREEN );
		BSP_LCD_FillCircle(x, y, 40);
	}

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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // above: MX_SDMMC1_SD_Init(); has to be commented out to not be called



  /* LCD Init */
	if (BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE) == LCD_ERROR)
	{
		/* Initialization Error */
		Error_Handler();
	}

	// the lcd bsp includes the fontNN.c files in the Utilities directory
	// in that file, a struct is declared: FontNN
	//BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetFont(&Font24);

	// the display is 240 px tall, XYZ px wide
	BSP_LCD_DisplayStringAt(0, 240 - 65, (uint8_t *)"Purple Ball", CENTER_MODE);



	// touchscreen init
	if(BSP_TS_InitEx(BSP_LCD_GetXSize(), BSP_LCD_GetYSize(), LCD_ORIENTATION_LANDSCAPE) != TS_OK) {
		Error_Handler();
	}

	BSP_LCD_ScreenDimmingConfig(100, 5, 1, 20);



	// LED
	BSP_LED_Init(LED1_PIN);
	BSP_LED_Init(LED2_PIN);
	BSP_LED_Off( LED1_PIN ); // orange labeled LD1
	BSP_LED_Off( LED2_PIN ); // doesn't work



	// Start timer
	HAL_TIM_Base_Start_IT(&htim6);

	//ResetAndDetectSensor(1); // EK TODO: the parameter SetDisplay doesn't matter

	// EK test1
	HAL_StatusTypeDef status;
	uint8_t pData;
	status = HAL_I2C_Mem_Read(&hi2c1, 0x52, 0xC0, 1, &pData, 1, HAL_TIMEOUT);
	// pData should be 0xEE
	if(status == HAL_OK) {
		printf("good");
	}
	//--

	// EK test2
	VL53L0X_Dev_t *pDev;
	pDev = &VL53L0XDev;
	pDev->I2cDevAddr = 0x52;
	pDev->Present = 0;

	int status2 = VL53L0X_DataInit(pDev);
	if( status2 == 0 ){
			pDev->Present = 1;
	}
	else{
			printf("VL53L0X_DataInit fail\n");
	}
	printf("VL53L0X %d Present and initiated to final 0x%x\n", pDev->Id, pDev->I2cDevAddr);
	pDev->Present = 1;
	//--

	// EK test3
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	//int range;

	// Initialize the device in continuous ranging mode
	VL53L0X_StaticInit(pDev);
	VL53L0X_PerformRefCalibration(pDev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(pDev, &refSpadCount, &isApertureSpads);
	//VL53L0X_SetInterMeasurementPeriodMilliSeconds(pDev, 250); // Program continuous mode Inter-Measurement period in milliseconds
	//VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	VL53L0X_SetInterMeasurementPeriodMilliSeconds(pDev, TIMED_RANGING_PERIOD);
	VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);

	// Start continuous ranging
	VL53L0X_StartMeasurement(pDev);

	/*
	for(uint8_t i=0; i<5; i++) {
		VL53L0X_GetRangingMeasurementData(pDev, &RangingMeasurementData);
		range = (int)RangingMeasurementData.RangeMilliMeter/10;
		HAL_Delay(100);
	}

	// Stop continuous ranging
	VL53L0X_StopMeasurement(pDev);

	// Ensure device is ready for other commands
	WaitStopCompleted(pDev);
	*/
	//--


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  	// this has to be the 1st thing in the loop
  	if(gone_sleep) {
			//HAL_ResumeTick();
			awakeFromSleep();
		}

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



  	if(HAL_GetTick()-last_sample >= TIMED_RANGING_PERIOD) {

  		// Get ADC value
  		// "10-bit ADC, divide the ADC output by 2 for the range in inches."
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			raw = HAL_ADC_GetValue(&hadc1);
			raw_in = (float)raw/2.0;
			raw_mm = raw_in*2.54*10;


			VL53L0X_GetRangingMeasurementData(pDev, &RangingMeasurementData);
			range = RangingMeasurementData.RangeMilliMeter/10; // cm
			range_mm = RangingMeasurementData.RangeMilliMeter; // mm

			// ----- comms

			if(range_mm < 400) { // tof stop at 40 cm (self-imposed limit)
				sprintf((char*)buf, "%d;", range_mm);
				HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_SET);
			} else if(raw_mm < 1200) { // sonar stop at 1.2 m (self-imposed limit)
				sprintf((char*)buf, "%d;", (int)raw_mm);
				HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_SET);
			} else {
				sprintf((char*)buf, "%d;", -1);
				HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_RESET);
			}
			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);

			// Send out buffer (temperature or error message)
			  		//sprintf((char*)buf, "hi there\r\n");
			//  		sprintf((char*)buf,
			//  		              "%u.%u C\r\n",
			//  		              ((unsigned int)temp_c / 100),
			//  		              ((unsigned int)temp_c % 100));
			  		//HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
			//HAL_UART_Transmit(&huart1, "~", 1, HAL_MAX_DELAY);



			// ------ avging

			// check it's in range
			if(range_mm < 400) {
				range_mm_sum += range_mm;
				sum_count++;
			} else {
				// to keep up with the timing
				sum_skip++;
			}

			if( (sum_count+sum_skip) >= 20) { // 20 because 1 sample / 50 ms = 20 samples / s (1000/50=20)
				range_mm_avg = (float)range_mm_sum / (float)sum_count;

				sum_count = 0;
				sum_skip = 0;
				range_mm_sum = 0;

				if(sample_index < num_samples && sample_index < 120) {
					all_samples[sample_index] = range_mm_avg;
					sample_index++;
				} else {
					// TODO: process all samples
				}

			}

			last_sample = HAL_GetTick();

		}



  	// imagine the case where gettick has overflowed, but last_ts has not
		// eg 100-30000
		// abs could be used to prevent this from being a negative number, however
		// in this case, it is not needed, because two unsigned integers being
		// subtracted results in an unsigned integer
		if( HAL_GetTick()-last_ts >= 1000 && dimmed_screen == false) {
			BSP_LCD_ScreenDimmingConfig(100, 5, 1, 20);
			BSP_LCD_ScreenDimmingOn();
			dimmed_screen = true;
		}


		uint8_t circle_x = 100;
		uint8_t circle_y = 100;
		drawCircle(circle_x, circle_y);

		BSP_TS_GetState(&ts_result);

		uint8_t num_touches = ts_result.touchDetected;

		for(int i=0; i<num_touches; i++) {
			Point touch;
			touch.X = ts_result.touchX[i];
			touch.Y = ts_result.touchY[i];

			if(dimmed_screen) {
				//BSP_LCD_ScreenDimmingConfig(5, 100, 1, 20);
				//BSP_LCD_ScreenDimmingOn();
				BSP_LCD_ScreenDimmingOff();
				dimmed_screen = false;
			}

			last_ts = HAL_GetTick();

			if(touch.X < circle_x+70 && touch.X > circle_x-70) {
				if(touch.Y < circle_y+70 && touch.Y > circle_y-70) {
					circle_selected = !circle_selected;
					BSP_LCD_Clear(LCD_COLOR_WHITE);
					break;
				}
			}

			BSP_LCD_SetTextColor( LCD_COLOR_BLUE );
			BSP_LCD_FillCircle(touch.X, touch.Y, 10);

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

	/* Reduce the System clock to below 2 MHz */
	SystemClock_Decrease();

	/* Suspend Tick increment to prevent wakeup by Systick interrupt.         */
	/* Otherwise the Systick interrupt will wake up the device within 1ms     */
	/* (HAL time base).                                                       */
	HAL_SuspendTick();

	/* De-init LED1 */
	/*BSP_LED_DeInit(LED1);*/

	/* Enter Sleep Mode, wake up is done once User push-button is pressed */
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);

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
//		gone_sleep = true;
//		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH); // JOY_SEL on SYS_WKUP2
//		prepareForSleep();
//		enterSleep();

		// simpler version
		gone_sleep = true;
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE); // left button configured as event
	}

	// LEFT wakes it up as an event

	if(GPIO_Pin == JOY_RIGHT_Pin) {
//		awakeFromSleep();

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
