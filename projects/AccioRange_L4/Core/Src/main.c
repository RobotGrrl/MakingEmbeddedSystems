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
// be sure to add these directories to the build paths in project properties
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
#define TIMED_RANGING_PERIOD    50    // in ms
#define LED_TOGGLE_DELAY        100
#define PWR_ANALYSIS 						1
#define PWR_ANALYSIS_DELAY			1000
#define RANGER_SAMPLE_RATE      20    // samples per second
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// touch screen related
TS_StateTypeDef ts_result;
uint32_t last_ts;

// states related
bool SAMPLE_SENSOR = false;
bool DIMMED_SCREEN = false;
bool SLEEP_MODE_ACTIVE = false;
bool ENTER_SLEEP_MODE = false;
bool BT_ENABLED = true;

// led related
bool led_on = false;
static uint32_t TimingDelay;

// sampling related
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


// ui related // UI234
struct Bubble {
   uint16_t x;
   uint16_t y;
   uint16_t radius;
   uint16_t hit_diameter;
   bool selected;
   uint16_t colour_active;
   uint16_t colour_inactive;
   bool redraw;
   uint8_t type;
   uint32_t last_selected;
};
uint8_t num_bubbles = 4;
struct Bubble ui_bubbles[4];
bool bubble_label_redraw = true;


// tof related
VL53L0X_RangingMeasurementData_t RangingMeasurementData; // global ranging struct

VL53L0X_Dev_t VL53L0XDev = {
		.Id=0,
		.DevLetter='e',
		.I2cHandle=&hi2c1,
		.I2cDevAddr=0x29 // EK: documentation says address is 0x29, why did the code say 0x52?
};

uint8_t VhvSettings;
uint8_t PhaseCal;
uint32_t refSpadCount;
uint8_t isApertureSpads;


// sampling related
// RAN_

struct Ranger {
	uint16_t max;
	uint16_t min;

	uint16_t raw;
	float raw_mm;

	float raw_sum;
	uint16_t sum_count;
	uint16_t sum_skip;

	float mm_avg;

	uint16_t record_index;
	float records[300];
	bool process_records;
	float record_sum;

	float val;
};
uint8_t num_rangers = 2;
struct Ranger rangers[2];

uint16_t NUM_RECORDS = 5;
uint16_t MAX_RECORDS = 300; // =5mins*60seconds

float sonar_mm_avg = 0.0;
float tof_mm_avg = 0.0;

float result_distance = 0.0;
bool UPDATE_RESULT = false;
float alpha = 0.75; // weight of tof


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

// sleep mode related
void reconfigureFromSleep(void);
void awakeFromSleep(void);
void enterSleep(void);

// ui related
void uiSetup(void);
void drawBubble(struct Bubble *bubble);
void deselectBubbles(uint8_t skip);

// tof related
void tofTestRegisterRead(void);
void tofInit(void);

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if(htim == &htim6) {
		HAL_GPIO_TogglePin(ARD_D7_GPIO_Port, ARD_D7_Pin); // test point for profiling
		SAMPLE_SENSOR = true;
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

  // lcd init
	if (BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE) == LCD_ERROR) {
		Error_Handler();
	}

	// touchscreen init
	if(BSP_TS_InitEx(BSP_LCD_GetXSize(), BSP_LCD_GetYSize(), LCD_ORIENTATION_LANDSCAPE) != TS_OK) {
		Error_Handler();
	}

	// LED GPIO
	BSP_LED_Init(LED2_PIN); // LD1
	BSP_LED_On(LED2_PIN); // LD1 orange turns on
	HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET); // LD2 green turns on
	// LD3 is attached to ARD_D13, which is not an output, it's attached to SPI
	HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_RESET); // laser
	HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, GPIO_PIN_RESET); // test point: timer frequency
	HAL_GPIO_WritePin(ARD_D2_GPIO_Port, ARD_D2_Pin, GPIO_PIN_SET); // turn on BT pwr transistor


	// tof related
	VL53L0X_Dev_t *pDev;
	pDev = &VL53L0XDev;
	tofTestRegisterRead();
	//tofInit();

	pDev->I2cDevAddr = 0x52;
	pDev->Present = 0;

	int status = VL53L0X_DataInit(pDev);
	if(status == 0) {
			pDev->Present = 1;
	} else {
			printf("VL53L0X_DataInit fail\n");
			return;
	}
	printf("VL53L0X %d Present and initiated to final 0x%x\n", pDev->Id, pDev->I2cDevAddr);
	pDev->Present = 1;

	// Initialize the device in continuous ranging mode
	VL53L0X_StaticInit(pDev);
	VL53L0X_PerformRefCalibration(pDev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(pDev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetInterMeasurementPeriodMilliSeconds(pDev, TIMED_RANGING_PERIOD);
	VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);
	VL53L0X_StartMeasurement(pDev);


	// sampling related
	for(uint8_t i=0; i<num_rangers; i++) {
		struct Ranger *r = &rangers[i];
		r->max = 0;
		r->min = 0;
		r->raw = 0;
		r->raw_mm = 0.0;
		r->raw_sum = 0.0;
		r->sum_count = 0;
		r->sum_skip = 0;
		r->mm_avg = 0.0;
		r->record_index = 0;
		for(uint16_t j=0; j<MAX_RECORDS; j++) {
			r->records[j] = 0.0;
		}
		r->process_records = false;
		r->record_sum = 0.0;
		r->val = 0.0;
	}
	rangers[0].max = 3000; // mm sonar
	rangers[0].min = 150;
	rangers[1].max = 500; // mm tof
	rangers[1].min = 10;


	// ui related
	uiSetup();


	// start timer
  HAL_TIM_Base_Start_IT(&htim6);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  	// wake from event, the first thing seen after exiting sleep mode
  	if(SLEEP_MODE_ACTIVE) {
			reconfigureFromSleep();
			awakeFromSleep();
			// refresh ui
			for(uint8_t i=0; i<num_bubbles; i++) {
				struct Bubble *b = &ui_bubbles[i];
				b->redraw = true;
			}
			bubble_label_redraw = true;
		}


  	// flag set from interrupt to enter in to sleep mode
  	if(ENTER_SLEEP_MODE) {
  		enterSleep();
  		HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE); // left button configured as event
  	}




  	// processing the sensor data
  	// sampling every 50 ms
  	// flag set from timer
  	if(SAMPLE_SENSOR) {

  		// get sonar adc value
			// "10-bit ADC, divide the ADC output by 2 for the range in inches."
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			rangers[0].raw = HAL_ADC_GetValue(&hadc1);
			rangers[0].raw_mm = (float)(rangers[0].raw/2.0)*2.54*10;

			// get time of flight i2c value
			VL53L0X_GetRangingMeasurementData(pDev, &RangingMeasurementData);
			rangers[1].raw_mm = RangingMeasurementData.RangeMilliMeter; // mm

			// RAN_

			for(uint8_t i=0; i<num_rangers; i++) {
				struct Ranger *r = &rangers[i];

				// clamp the values between a min and max
				// if it doesn't fit, skip it
				if(r->raw_mm < r->max && r->raw_mm > r->min) {
					r->raw_sum += r->raw_mm;
					r->sum_count++;
					HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_SET); // laser on
				} else {
					r->sum_skip++;
					HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_RESET); // laser off
				}

				uint16_t total_sum = r->sum_count + r->sum_skip;

				// check for when the count matches the sample rate
				if(total_sum >= RANGER_SAMPLE_RATE) {

					if(r->sum_count > 0) {
						r->mm_avg = r->raw_sum / (float)r->sum_count; // get the average
					} else {
						r->mm_avg = 0.0; // clamp to 0 if there was nothing added
					}

					// reset the counters
					r->raw_sum = 0;
					r->sum_count = 0;
					r->sum_skip = 0;

					// see if there's enough records ready to make the readout
					if(r->record_index < NUM_RECORDS && r->record_index < MAX_RECORDS) {
						r->records[r->record_index] = r->mm_avg;
						r->record_index++;
					} else {
						r->process_records = true;
					}

				}

				// update our live variables
				if(i==0) {
					sonar_mm_avg = r->mm_avg;
				} else if(i==1) {
					tof_mm_avg = r->mm_avg;
				}

			} // end of rangers loop

			// hope all of this takes < 50 ms

  		SAMPLE_SENSOR = false; // set to true by timer
  	}






  	// process the records
  	bool ready_to_process = false;
  	uint8_t ready_count = 0;
  	for(uint8_t i=0; i<num_rangers; i++) {
  		struct Ranger *r = &rangers[i];
  		if(r->process_records == true) {
  			ready_count++;
  		}
  	}
  	if(ready_count == num_rangers) {
  		ready_to_process = true;
  	}
  	if(ready_to_process) {

  		for(uint8_t i=0; i<num_rangers; i++) {
				struct Ranger *r = &rangers[i];

				for(uint16_t j=0; j<r->record_index; j++) {
					r->record_sum += r->records[j];
				}

				// record average
				r->val = r->record_sum/r->record_index;

				// reset counters
				r->record_index = 0;
				r->record_sum = 0.0;

				// reset this flag
				r->process_records = false;
			}

  		// hooray! we have the value!
  		result_distance = (alpha*rangers[1].val) + ( (1-alpha)*rangers[0].val);
  		UPDATE_RESULT = true;

  	}




  	// send the result distance over bt to our app
  	if(UPDATE_RESULT) {

			// this should already be on, but let's do it again just in case...
			// turn on BT pwr transistor
  		HAL_GPIO_WritePin(ARD_D2_GPIO_Port, ARD_D2_Pin, GPIO_PIN_SET);

  		// send to uart
  		if(result_distance > 0) {
				sprintf((char*)buf, "%d;", (uint16_t)result_distance);
			} else {
				sprintf((char*)buf, "%d;", -1);
			}
			HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), HAL_MAX_DELAY);

  		// update lcd
			// TODO

  		UPDATE_RESULT = false;
  	}





		// update ui
		for(uint8_t i=0; i<num_bubbles; i++) {
			drawBubble(&ui_bubbles[i]);
		}
		if(bubble_label_redraw) {
			BSP_LCD_SetTextColor( LCD_COLOR_DARKGREEN );
			BSP_LCD_SetFont(&Font24);
			//BSP_LCD_DisplayStringAt(0, 90, (uint8_t *)" 10   30   90", LEFT_MODE);
			BSP_LCD_DisplayStringAt(0, 90, (uint8_t *)" 2   5   10", LEFT_MODE);
			bubble_label_redraw = false;
		}

		// update ts
		BSP_TS_GetState(&ts_result);
		uint8_t num_touches = ts_result.touchDetected;

		// wake up the screen on touch
		if(num_touches > 0 && DIMMED_SCREEN == true) {
			BSP_LCD_ScreenDimmingOff();
			DIMMED_SCREEN = false;
			last_ts = HAL_GetTick();
		}

		// hit testing selected bubbles
		for(uint8_t i=0; i<num_touches; i++) {
			Point touch;
			touch.X = ts_result.touchX[i];
			touch.Y = ts_result.touchY[i];

			for(uint8_t j=0; j<num_bubbles; j++) {
				struct Bubble *b = &ui_bubbles[j];

				// hit testing
				if(touch.X < b->x + b->hit_diameter && touch.X > b->x - b->hit_diameter) {
					if(touch.Y < b->y + b->hit_diameter && touch.Y > b->y - b->hit_diameter) {

						if(b->type == 1) { // ui bubbles

							if(!b->selected) { // not selected prior
								b->selected = !b->selected;
								deselectBubbles(j); // "single touch"
								b->redraw = true;
								b->last_selected = HAL_GetTick();

								// update the number of records
								if(j==0) {
									NUM_RECORDS = 2;
								} else if(j==1) {
									NUM_RECORDS = 5;
								} else if(j==2) {
									NUM_RECORDS = 10;
								}

							}

						} else if(b->type == 2) { // go bubble

							if(HAL_GetTick()-b->last_selected > 80) { // 80 ms debounce
								b->selected = !b->selected;
								b->redraw = true;
								b->last_selected = HAL_GetTick();
							}

						}

					}
				} // hit testing end

			} // bubble loop end

		} // touches end


		// check for last touch
		// imagine the case where gettick has overflowed, but last_ts has not
		// eg 100-30000
		// abs could be used to prevent this from being a negative number, however
		// in this case, it is not needed, because two unsigned integers being
		// subtracted results in an unsigned integer
		if( HAL_GetTick()-last_ts >= 5000 && DIMMED_SCREEN == false) {
			//BSP_LCD_ScreenDimmingConfig(100, 5, 5, 20); // 100-5=95/5=19*20=380ms
			BSP_LCD_ScreenDimmingConfig(100, 5, 5, 1); // 100-5=95/5=19*20=380ms
			BSP_LCD_ScreenDimmingOn();
			DIMMED_SCREEN = true;
		}


		// led heartbeat
		if(TimingDelay == 0) {
			if(led_on) {
				BSP_LED_On(LED2_PIN); // LD1 orange
			} else {
				BSP_LED_Off(LED2_PIN); // LD1 orange
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

void tofTestRegisterRead(void) {
	HAL_StatusTypeDef status;
	uint8_t pData;
	status = HAL_I2C_Mem_Read(&hi2c1, 0x52, 0xC0, 1, &pData, 1, HAL_TIMEOUT);
	// pData should be 0xEE
	if(status == HAL_OK) {
		printf("good");
	}
}

void tofInit(void) {
	/*
	pDev->I2cDevAddr = 0x52;
	pDev->Present = 0;

	int status = VL53L0X_DataInit(pDev);
	if(status == 0) {
			pDev->Present = 1;
	} else {
			printf("VL53L0X_DataInit fail\n");
			return;
	}
	printf("VL53L0X %d Present and initiated to final 0x%x\n", pDev->Id, pDev->I2cDevAddr);
	pDev->Present = 1;

	// Initialize the device in continuous ranging mode
	VL53L0X_StaticInit(pDev);
	VL53L0X_PerformRefCalibration(pDev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(pDev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetInterMeasurementPeriodMilliSeconds(pDev, TIMED_RANGING_PERIOD);
	VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);
	VL53L0X_StartMeasurement(pDev);
	*/
}




// UI123

void drawBubble(struct Bubble *bubble) {

	if(!bubble->redraw) return;

	if(!bubble->selected) {
		BSP_LCD_SetTextColor( bubble->colour_active );
		BSP_LCD_FillCircle(bubble->x, bubble->y, bubble->radius);
	} else {
		BSP_LCD_SetTextColor( bubble->colour_inactive );
		BSP_LCD_FillCircle(bubble->x, bubble->y, bubble->radius);
	}

	bubble->redraw = false;

}

void deselectBubbles(uint8_t skip) {

	for(uint8_t i=0; i<num_bubbles; i++) {
		struct Bubble *b = &ui_bubbles[i];
		if(b->type == 1) { // ui bubbles
			if(i == skip) continue;
			b->selected = false;
			b->redraw = true;
		}
	}

}

void uiSetup(void) {

	uint16_t radius = 30;
	uint16_t hit_diameter = 35;
	uint16_t y = 50;

	// left bubble
	ui_bubbles[0].x = 40;
	ui_bubbles[0].y = y;
	ui_bubbles[0].radius = radius;
	ui_bubbles[0].hit_diameter = hit_diameter;
	ui_bubbles[0].selected = false;
	ui_bubbles[0].colour_active = LCD_COLOR_CYAN;
	ui_bubbles[0].colour_inactive = LCD_COLOR_GREEN;
	ui_bubbles[0].redraw = true;
	ui_bubbles[0].type = 1;
	ui_bubbles[0].last_selected = 0;

	// middle bubble
	ui_bubbles[1].x = 120;
	ui_bubbles[1].y = y;
	ui_bubbles[1].radius = radius;
	ui_bubbles[1].hit_diameter = hit_diameter;
	ui_bubbles[1].selected = true;
	ui_bubbles[1].colour_active = LCD_COLOR_LIGHTBLUE;
	ui_bubbles[1].colour_inactive = LCD_COLOR_GREEN;
	ui_bubbles[1].redraw = true;
	ui_bubbles[1].type = 1;
	ui_bubbles[1].last_selected = 0;

	// right bubble
	ui_bubbles[2].x = 200;
	ui_bubbles[2].y = y;
	ui_bubbles[2].radius = radius;
	ui_bubbles[2].hit_diameter = hit_diameter;
	ui_bubbles[2].selected = false;
	ui_bubbles[2].colour_active = LCD_COLOR_LIGHTMAGENTA;
	ui_bubbles[2].colour_inactive = LCD_COLOR_GREEN;
	ui_bubbles[2].redraw = true;
	ui_bubbles[2].type = 1;
	ui_bubbles[2].last_selected = 0;

	// go bubble
	ui_bubbles[3].x = 120;
	ui_bubbles[3].y = 170;
	ui_bubbles[3].radius = 45;
	ui_bubbles[3].hit_diameter = 50;
	ui_bubbles[3].selected = true;
	ui_bubbles[3].colour_active = LCD_COLOR_GREEN;
	ui_bubbles[3].colour_inactive = LCD_COLOR_BLUE;
	ui_bubbles[3].redraw = true;
	ui_bubbles[3].type = 2;
	ui_bubbles[3].last_selected = 0;

}








void reconfigureFromSleep(void) {
	/* System is Low Power Run mode when exiting Low Power Sleep mode,
		 disable low power run mode and reset the clock to initialization configuration */
	HAL_PWREx_DisableLowPowerRunMode();

	/* Configure the system clock for the RUN mode */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* Re-init GPIOs */
	MX_GPIO_Init();

	/* Resume Tick interrupt if disabled prior to Low Power Sleep mode entry */
	HAL_ResumeTick();
}


void awakeFromSleep(void) {

	// turn on LD1, LD2
	BSP_LED_On(LED2_PIN); // LD1 orange turns on
	HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET); // LD2 green turns on
	if(PWR_ANALYSIS) HAL_Delay(PWR_ANALYSIS_DELAY);

	// turn on lcd
	BSP_LCD_DisplayOn();
	DIMMED_SCREEN = false;
	if(PWR_ANALYSIS) HAL_Delay(PWR_ANALYSIS_DELAY);

	// turn on laser and test point
	HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_SET); // laser
	HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, GPIO_PIN_SET); // test point: timer frequency
	if(PWR_ANALYSIS) HAL_Delay(PWR_ANALYSIS_DELAY);

	// turn on ts
	BSP_TS_ITConfig();
	if(PWR_ANALYSIS) HAL_Delay(PWR_ANALYSIS_DELAY);

	// turn on BT pwr transistor
	HAL_GPIO_WritePin(ARD_D2_GPIO_Port, ARD_D2_Pin, GPIO_PIN_SET);
	if(PWR_ANALYSIS) HAL_Delay(PWR_ANALYSIS_DELAY);

	// set flags
	BT_ENABLED = true;
	ENTER_SLEEP_MODE = false;
	SLEEP_MODE_ACTIVE = false;

}


void enterSleep(void) {

	// turn off ts
	BSP_TS_ITDeConfig();

	// turn off lcd
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	if(DIMMED_SCREEN == true) {
		BSP_LCD_ScreenDimmingOff();
	}
	BSP_LCD_ScreenDimmingConfig(100, 0, 5, 20);
	BSP_LCD_ScreenDimmingOn();
	BSP_LCD_DisplayOff();
	DIMMED_SCREEN = true;
	HAL_Delay(1000); // wait for fade to finish
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_RESET); // force the backlight off

	// turn off BT pwr transistor
	HAL_GPIO_WritePin(ARD_D2_GPIO_Port, ARD_D2_Pin, GPIO_PIN_RESET);
	if(PWR_ANALYSIS) HAL_Delay(PWR_ANALYSIS_DELAY);

	// turn off LD1, LD2
	BSP_LED_Off(LED2_PIN); // LD1 orange turns off
	HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET); // LD2 green turns off
	if(PWR_ANALYSIS) HAL_Delay(PWR_ANALYSIS_DELAY);

	// turn off laser and test point
	HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_RESET); // laser
	HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, GPIO_PIN_RESET); // test point: timer frequency
	if(PWR_ANALYSIS) HAL_Delay(PWR_ANALYSIS_DELAY);

	// set flags
	BT_ENABLED = false;
	ENTER_SLEEP_MODE = false;
	SLEEP_MODE_ACTIVE = true;

}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	// DOWN: enter sleep mode
	// LEFT: wake up (event)
	// RIGHT: toggle bt enabled

	if(GPIO_Pin == JOY_DOWN_Pin) {
		// set flag to enter sleep mode
		ENTER_SLEEP_MODE = true;
	}

	if(GPIO_Pin == JOY_RIGHT_Pin) {

		if(BT_ENABLED) {
			HAL_GPIO_WritePin(ARD_D2_GPIO_Port, ARD_D2_Pin, GPIO_PIN_RESET); // turn off BT pwr transistor
		} else {
			HAL_GPIO_WritePin(ARD_D2_GPIO_Port, ARD_D2_Pin, GPIO_PIN_SET); // turn on BT pwr transistor
		}

		BT_ENABLED = !BT_ENABLED;

	}

}


/**
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  //HAL_IncTick(); // it's already incremented in _it.c
  if (TimingDelay != 0) {
    TimingDelay--;
  } else {
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
