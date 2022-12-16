/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/**
 * How does this code function?
 * This section serves as notes and planning for a very lazy developer.  Here's the general idea
 * When the device starts up, it checks the compass's current direction.  It then draws a vector
 * pointing in that direction and determines which motors to vibrate (max 3) and turns them on.
 *
 * For now thats it.  Later can try interesting things with changing the vibration period and tone.
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

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


typedef struct {
	float x;
	float y;
} compass_vector_t;


typedef struct __attribute__((packed)) {
	int16_t X;
	int16_t Y;
	int16_t Z;
} magnetometer_reading_t;

typedef enum {
	SINGLE_BLINK,
	DOUBLE_BLINK,
} blink_type_t;

// register definitions
#define MAG_I2C_ID (0x1E << 1)
#define MAG_REG_CFG_A (0x60)
#define MAG_REG_CFG_C (0x62)
#define MAG_REG_OFFSETX_L (0x45)
#define MAG_REG_OUTX_L (0x68)

#define IMU_I2C_ID (0x6A << 1)
#define IMU_REG_WHOAMI (0x0F)
#define IMU_REG_MD1_CFG (0x5E)
#define IMU_REG_TAP_CFG (0x58)
#define IMU_REG_CTRL1_XL (0x10)
#define IMU_REG_TAPSRC (0x1C)
#define IMU_REG_FUNC_CFG_ACCESS (0x01)
#define IMU_REG_FIFO_CTRL5 (0x0A)

#define IMU_MD1_CFG (0b00001000)
#define IMU_TAP_CFG (0b10001111)  // TODO remove last bit as latch
#define IMU_CTRL1_XL (0b01000000)
#define IMU_FUNC_CFG_ACCESS (0b10100000)
#define IMU_FIFO_CTRL5 (0b00100110)

static magnetometer_reading_t mag_offset = {
	.X = -136,
	.Y = -352,
	.Z = -216
};

GPIO_TypeDef* motor_ports[] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t motor_pins[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
unsigned int target_buzzer = 0;

#define LED_PORT GPIOB
#define LED_PIN GPIO_PIN_0

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void init_mag(void);
static void init_imu(void);
static void get_compass_direction(compass_vector_t *direction);
static void set_buzzers(compass_vector_t *direction);
static void error_blink(blink_type_t blink_type);
static void set_mag_offset(void);
static void motor_test(void);
static void configure_charger(void);
static void configure_motors(void);

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
/*
  HAL_NVIC_DisableIRQ(SysTick_IRQn);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_NVIC_EnableIRQ(SysTick_IRQn);
*/

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  char *start_string = "belt system initialized!\n";
  HAL_UART_Transmit(&huart1, start_string, sizeof(start_string), 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // nice linky: https://www.youtube.com/watch?v=VfbW6nfG4kw
  // may need to set NVIC enable on the IRQ for the timer: https://electronics.stackexchange.com/questions/544797/output-compares-triggering-function-not-work-on-stm32

  compass_vector_t direction;
  init_mag();
  set_mag_offset();
  init_imu();
  configure_motors();
  configure_charger();
  //motor_test();

  HAL_TIM_Base_Start_IT(&htim16);


  while (1)
  {
	get_compass_direction(&direction);
	configure_motors();
	//set_buzzers(&direction);
	HAL_Delay(500);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void init_mag(void) {
	// nominal startup, consult the datasheet
	uint8_t cfga_nominal[] = {0x80};
	uint8_t cfgc_nominal[] = {0x01};

	HAL_StatusTypeDef stat_a = HAL_I2C_Mem_Write(&hi2c2, MAG_I2C_ID, MAG_REG_CFG_A, 1, cfga_nominal, sizeof(cfga_nominal), 1000);
	HAL_StatusTypeDef stat_c = HAL_I2C_Mem_Write(&hi2c2, MAG_I2C_ID, MAG_REG_CFG_C, 1, cfgc_nominal, sizeof(cfga_nominal), 1000);

	if(stat_a != HAL_OK || stat_c != HAL_OK) {
		error_blink(DOUBLE_BLINK);
	}
}

static void set_mag_offset(void) {
	HAL_StatusTypeDef stat_a = HAL_I2C_Mem_Write(&hi2c2, MAG_I2C_ID, MAG_REG_OFFSETX_L, 1, &mag_offset, sizeof(mag_offset), 1000);
}

static HAL_StatusTypeDef imu_write_reg(uint8_t reg, uint8_t value) {
	uint8_t val[] = {value};
	HAL_StatusTypeDef stat = HAL_I2C_Mem_Read(&hi2c2, IMU_I2C_ID, reg, 1, val, sizeof(val), 1000);
	return stat;
}

static void init_imu(void) {
	uint8_t ID[] = {0x00};
	HAL_StatusTypeDef stat = HAL_I2C_Mem_Read(&hi2c2, IMU_I2C_ID, IMU_REG_WHOAMI, 1, ID, sizeof(ID), 1000);

	HAL_StatusTypeDef stat_md1 = imu_write_reg(IMU_REG_MD1_CFG, IMU_MD1_CFG);
	HAL_StatusTypeDef stat_tap = imu_write_reg(IMU_REG_TAP_CFG, IMU_TAP_CFG);
	HAL_StatusTypeDef stat_nom = imu_write_reg(IMU_REG_CTRL1_XL, IMU_CTRL1_XL);
	HAL_StatusTypeDef stat_fifo = imu_write_reg(IMU_REG_FIFO_CTRL5, IMU_FIFO_CTRL5);

	if((stat | stat_md1 | stat_tap | stat_nom ) != HAL_OK) {
		error_blink(DOUBLE_BLINK);
	}
}

static void error_blink(blink_type_t blink_type) {
	switch(blink_type) {
		case DOUBLE_BLINK:
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}

static void get_compass_direction(compass_vector_t *direction) {
	static magnetometer_reading_t reading;
	HAL_I2C_Mem_Read(&hi2c2, MAG_I2C_ID, MAG_REG_OUTX_L, 1, (uint8_t *)&reading, sizeof(reading), HAL_MAX_DELAY);

	// lazy change of coordinate frames, probably wrong
	direction->x = (float)reading.X;
	direction->y = (float)reading.Z;
	float x = direction->x;
	float y = direction->y;

	// bad cohesion!  Probably break this stuff out into vector management functions
	float length = sqrt(x*x + y*y);
	if(abs(length) < 0.001) {
		direction->y = 1.0;
		direction->x = 0.0;
	} else {
		direction->x /= length;
		direction->y /= length;
	}

	static char message[64];
	int string_length = sprintf(message, "Got new reading: %f %f\n", direction->x, direction->y);
	HAL_UART_Transmit(&huart1, message, string_length, HAL_MAX_DELAY);
}


static void set_buzzers(compass_vector_t *direction) {
	// pick a buzzer based on the direction.

	// make angle from 0 to 8 with wrapping
	// in a better world, this wouldn't be coupled to the number of motors, but am lazy...
	float angle = (atan2(direction->x, direction->y) / (2.0*3.14159)) * 8.0;
	unsigned int new_target = 0;
	if(angle < 0.5 || angle > 7.5) {
		new_target = 0;
	} else {
		new_target = (unsigned int)(angle + 0.5);
	}
	HAL_TIM_Base_Stop_IT(&htim16);
	// probably clear timer or something
	HAL_GPIO_WritePin(motor_ports[target_buzzer], motor_pins[target_buzzer], GPIO_PIN_RESET);
	target_buzzer = new_target;
	HAL_TIM_Base_Start_IT(&htim16);

	static char message[64];
	int string_length = sprintf(message, "Setting motor: %d\n", new_target);
	HAL_UART_Transmit(&huart1, message, string_length, HAL_MAX_DELAY);
}

static void motor_test(void) {
	int num_motors = sizeof(motor_ports) / sizeof(motor_ports[0]);
	for(int i = 0; i < num_motors; i++) {
		HAL_GPIO_WritePin(motor_ports[i], motor_pins[i], GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(motor_ports[i], motor_pins[i], GPIO_PIN_RESET);
		HAL_Delay(100);
	}
}

static void configure_motors(void) {
	int num_motors = sizeof(motor_ports) / sizeof(motor_ports[0]);
	for(int i = 0; i < num_motors; i++) {
		//HAL_GPIO_WritePin(motor_ports[i], motor_pins[i], GPIO);
		HAL_GPIO_TogglePin(motor_ports[i], motor_pins[i]);
	}
}

static void configure_charger(void) {
	HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim == &htim16) {

		//HAL_GPIO_WritePin(motor_ports[target_buzzer], motor_pins[target_buzzer], GPIO_PIN_SET);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim == &htim16) {
		//HAL_GPIO_WritePin(motor_ports[target_buzzer], motor_pins[target_buzzer], GPIO_PIN_RESET);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
