/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

#include "hcsr04.h"
#include "motors.h"

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
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

//L3GD20H
uint8_t l3gd20h_counter = 0;

uint8_t licznikOffsetu = 0;
uint8_t licznikOffsetu_nr = 200;
int wartoscSpoczynkowa;
uint8_t pierwszeDwiesciePom = 0;
long Zaxis_sum;
float Z_angularRate; //given in dps
#define SC 0.00875
volatile float deltaZ = 0;

int8_t liczbaObrotowPrawo = 0;
int8_t liczbaObrotowLewo = 0;

uint16_t katObrotu = 0;
/*
const int sample_no = 100; // no of samples for aproximation
int16_t ax, ay, az;
float x, y, z;
int  sample;
float angle_x, angle_y, angle_z;
long ax_sum, ay_sum, az_sum;
*/

//rejestry
#define L3GD20H_ADDRESS (0x6B << 1) // adres zyroskopu: 1101 011x
#define L3GD20H_CTRL1_A 0x20 // rejestr ustawien 1
#define L3GD20H_X_L_A 0x28 // mlodszy bajt danych osi X
//mlodszy bajt danych osi Z z njatsraszym bitem ustawionym na 1 w celu
//wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
#define L3GD20H_X_L_A_MULTI_READ (L3GD20H_X_L_A | 0x80)
//maski bitowe
// CTRL_REG1_A = [DR1][DR0][BW1][BW0][PD][ZEN][YEN][XEN]
#define L3GD20H_XYZ_ENABLE 0xF // 0000 1111
#define L3GD20H_100HZ 0x10 // 0001 0000
//zmienne
volatile uint8_t Data[6]; // Zmienna do bezposredniego odczytu z zyroskopu
volatile int16_t Xaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi X
volatile int16_t Yaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Y
volatile int16_t Zaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Z
volatile uint8_t Settings = L3GD20H_XYZ_ENABLE | L3GD20H_100HZ;

//UART
volatile uint8_t data[50];
volatile uint16_t size = 0;

//MOTORS
uint8_t setSpeedMotors;

uint8_t leftMotor = 0;
uint8_t rightMotor = 0;
uint8_t ninetyDegrees = 0;

uint32_t motorLeftCounter = 0;
uint32_t motorRightCounter = 0;
uint32_t distanceTraveled = 0;

volatile uint8_t wykonanoPomiar;

uint16_t timer_test;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) || (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
		{
			static uint16_t timeN;

			timeN = (uint16_t)((uint16_t)__HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_2)
							- (uint16_t)__HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1));

			hcsr04_N_distance = (float)timeN / 2.0 * 0.0343;

			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
		}

		if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) || (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4))
		{
			static int16_t timeW;

			timeW = (uint16_t)((uint16_t)__HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_4)
							- (uint16_t)__HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_3));

			if (((float)timeW / 2.0 * 0.0343) > 0)
			{
				hcsr04_W_distance = (float)timeW / 2.0 * 0.0343;
			}

			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
		}
	}

	if(htim->Instance == TIM5)
	{
		if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) || (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
		{
			static int16_t timeE;

			timeE = (uint16_t)((uint16_t)__HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_2)
							- (uint16_t)__HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_1));

			if (((float)timeE / 2.0 * 0.0343) > 0)
			{
				hcsr04_E_distance = (float)timeE / 2.0 * 0.0343;
			}

			HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
		}
	}
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Mem_Read_DMA(&hi2c1, L3GD20H_ADDRESS, L3GD20H_X_L_A_MULTI_READ, 1, Data, 6);

	pierwszeDwiesciePom++; // pominiecie 10 pierwszych pomiarów

	if(pierwszeDwiesciePom > 200)
	{
		pierwszeDwiesciePom = 201;

		Xaxis = ((Data[1] << 8) | Data[0]);
		Yaxis = ((Data[3] << 8) | Data[2]);
		Zaxis = ((Data[5] << 8) | Data[4]);

		if (wartoscSpoczynkowa == 0)
		{
			licznikOffsetu++;
			Zaxis_sum = Zaxis_sum + Zaxis;
			if (licznikOffsetu == licznikOffsetu_nr)
			{
				wartoscSpoczynkowa = Zaxis_sum / licznikOffsetu_nr;
				Zaxis_sum = 0;
			}
		}
		else
		{
			Z_angularRate = SC * (Zaxis - wartoscSpoczynkowa);

			if ((Z_angularRate > 5) || Z_angularRate < (-5))
			{
				deltaZ += (0.22 * 0.01 * Z_angularRate);
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM10)
	{
		//wys³anie ramki transmisjnej do PC
		size = sprintf(data, "N%06.2f W%06.2f E%06.2f D%05d EP%05d G%08.2f\n\r",
				hcsr04_N_distance, hcsr04_W_distance, hcsr04_E_distance,
				distanceTraveled, motorRightCounter,
				deltaZ);

		wykonanoPomiar = 0;

		HAL_UART_Transmit_DMA(&huart1, data, size);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((received[0] != 'o') && (received[0] != 'f')){
		if(received[0] == 'a') setSpeedMotors = 10;
		if(received[0] == 'b') setSpeedMotors = 20;
		if(received[0] == 'c') setSpeedMotors = 50;
	}

	if ((received[0] == 'o') || (received[0] == 'f'))
	{
		if(received[0] == 'o'){
			leftMotor = 1;
			liczbaObrotowPrawo++;
		}
		else{
			leftMotor = 0;
		}

		if(received[1] == 'o'){
			rightMotor = 1;
			liczbaObrotowLewo++;
		}
		else{
			rightMotor = 0;
		}
	}

	if (received[2] == 'd') ninetyDegrees = 1;

	if (received[0] == 'r'){
		distanceTraveled = 0;
		motorRightCounter = 0;
		//+resetowanie pomiary zyroskopu
		licznikOffsetu = 0;
		wartoscSpoczynkowa = 0;
		deltaZ = 0;
	}

	HAL_UART_Receive_DMA(&huart1, &received, 2); // Ponowne w³¹czenie nas³uchiwania
}

void rideMotorStraight(void);
void RideMotorReverse(void);
void rideMotorLeft(void);
void rideMotorRight(void);


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
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //l3gd20h
  HAL_I2C_Mem_Write_DMA(&hi2c1,L3GD20H_ADDRESS, L3GD20H_CTRL1_A, 1, &Settings, 1);
  HAL_Delay(10);
  HAL_I2C_Mem_Read_DMA(&hi2c1, L3GD20H_ADDRESS, L3GD20H_X_L_A_MULTI_READ, 1, Data, 6);

  //uart
  HAL_UART_Receive_DMA(&huart1, &received, 2);

  //timery
  HAL_TIM_Base_Start_IT(&htim10);

  //hcsr04
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);

  //testy 3 czujnika
  //HAL_TIM_Base_Start(&htim1);
  //HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  //HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);


  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  TIM4->CCR1 = 3;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if((leftMotor == 1) && (rightMotor == 1) && (ninetyDegrees == 0))
	  {
		  rideMotorStraight();
		  motorLeftCounter++;
		  motorRightCounter++;

		  distanceTraveled = ((float) motorLeftCounter / 512) * 20.3;

	  }

	  //obrot o 90 stopni w prawo
	  if((leftMotor == 1) && (rightMotor == 0))
	  {
		  //rideMotorRight();

		  if ((deltaZ <= ((liczbaObrotowPrawo*89)+(liczbaObrotowLewo*(-89))) && wykonanoPomiar == 0)) //10st
		  		  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ <= ((liczbaObrotowPrawo*89)+(liczbaObrotowLewo*(-89))) && wykonanoPomiar == 1)) //10st
		  		  		  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*89)+(liczbaObrotowLewo*(-89))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*178)+(liczbaObrotowLewo*(-178))))) //10st
		  	  	  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*89)+(liczbaObrotowLewo*(-89))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*178)+(liczbaObrotowLewo*(-178))))) //10st
		  		  	  	  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*178)+(liczbaObrotowLewo*(-178))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*267)+(liczbaObrotowLewo*(-267))))) //20st
		  	  	  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*178)+(liczbaObrotowLewo*(-178))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*267)+(liczbaObrotowLewo*(-267))))) //20st
		 		  	  	  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*267)+(liczbaObrotowLewo*(-267))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*356)+(liczbaObrotowLewo*(-356))))) //30st
		  	  	  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*267)+(liczbaObrotowLewo*(-267))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*356)+(liczbaObrotowLewo*(-356))))) //30st
		  		  	  	  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*356)+(liczbaObrotowLewo*(-356))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*445)+(liczbaObrotowLewo*(-445))))) //40st
		  		  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*356)+(liczbaObrotowLewo*(-356))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*445)+(liczbaObrotowLewo*(-445))))) //40st
		  		  		  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*445)+(liczbaObrotowLewo*(-445))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*534)+(liczbaObrotowLewo*(-534))))) //50st
		  		  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*445)+(liczbaObrotowLewo*(-445))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*534)+(liczbaObrotowLewo*(-534))))) //50st
		  		  		  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*534)+(liczbaObrotowLewo*(-534))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*623)+(liczbaObrotowLewo*(-623))))) //60st
		  		  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*534)+(liczbaObrotowLewo*(-534))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*623)+(liczbaObrotowLewo*(-623)))))//60st
		  		  		  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*623)+(liczbaObrotowLewo*(-623))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*712)+(liczbaObrotowLewo*(-712))))) //70st
		  	  	  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*623)+(liczbaObrotowLewo*(-623))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*712)+(liczbaObrotowLewo*(-712))))) //70st
		  		  	  	  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*712)+(liczbaObrotowLewo*(-712))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*801)+(liczbaObrotowLewo*(-801))))) //80st
		  	  	  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*712)+(liczbaObrotowLewo*(-712))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*801)+(liczbaObrotowLewo*(-801)))))//80st
		 		  	  	  	  { rideMotorRight(); }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*801)+(liczbaObrotowLewo*(-801))) && wykonanoPomiar == 0)
				  && (deltaZ <= ((liczbaObrotowPrawo*890)+(liczbaObrotowLewo*(-890))))) //80st
		  		  	  { HAL_Delay(1000); wykonanoPomiar = 1; }

		  else if ((deltaZ >= ((liczbaObrotowPrawo*801)+(liczbaObrotowLewo*(-801))) && wykonanoPomiar == 1)
				  && (deltaZ <= ((liczbaObrotowPrawo*890)+(liczbaObrotowLewo*(-890)))))//80st
		  		  		  	  { rideMotorRight(); }

		  else if(deltaZ > ((liczbaObrotowPrawo*890)+(liczbaObrotowLewo*(-890))))
		  {
			  leftMotor = 0;
			  rightMotor = 0;
		  }

	  }

	  //obrot o 90 stopni w lewo
	  if((leftMotor == 0) && (rightMotor == 1))
	  {
	  	  rideMotorLeft();

		  if(deltaZ < ((liczbaObrotowPrawo*890)+(liczbaObrotowLewo*(-890))))
		  {
			  leftMotor = 0;
			  rightMotor = 0;
		  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 128;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 83;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR_RIGHT_P1_Pin|MOTOR_RIGHT_P2_Pin|MOTOR_RIGHT_P3_Pin|MOTOR_RIGHT_P4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_P3_Pin|MOTOR_LEFT_P4_Pin|MOTOR_LEFT_P1_Pin|MOTOR_LEFT_P2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_RIGHT_P1_Pin MOTOR_RIGHT_P2_Pin MOTOR_RIGHT_P3_Pin MOTOR_RIGHT_P4_Pin */
  GPIO_InitStruct.Pin = MOTOR_RIGHT_P1_Pin|MOTOR_RIGHT_P2_Pin|MOTOR_RIGHT_P3_Pin|MOTOR_RIGHT_P4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BOARD_LED_Pin */
  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_LEFT_P3_Pin MOTOR_LEFT_P4_Pin MOTOR_LEFT_P1_Pin MOTOR_LEFT_P2_Pin */
  GPIO_InitStruct.Pin = MOTOR_LEFT_P3_Pin|MOTOR_LEFT_P4_Pin|MOTOR_LEFT_P1_Pin|MOTOR_LEFT_P2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void rideMotorStraight(void)
{
	  HAL_GPIO_WritePin(MOTOR_RIGHT_P4_GPIO_Port, MOTOR_RIGHT_P4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P1_GPIO_Port, MOTOR_LEFT_P1_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P1_GPIO_Port, MOTOR_RIGHT_P1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P4_GPIO_Port, MOTOR_LEFT_P4_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P1_GPIO_Port, MOTOR_RIGHT_P1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P4_GPIO_Port, MOTOR_LEFT_P4_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P2_GPIO_Port, MOTOR_RIGHT_P2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P3_GPIO_Port, MOTOR_LEFT_P3_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P2_GPIO_Port, MOTOR_RIGHT_P2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P3_GPIO_Port, MOTOR_LEFT_P3_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P3_GPIO_Port, MOTOR_RIGHT_P3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P2_GPIO_Port, MOTOR_LEFT_P2_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P3_GPIO_Port, MOTOR_RIGHT_P3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P2_GPIO_Port, MOTOR_LEFT_P2_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P4_GPIO_Port, MOTOR_RIGHT_P4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P1_GPIO_Port, MOTOR_LEFT_P1_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);
}

void rideMotorReverse(void)
{
	HAL_GPIO_WritePin(MOTOR_RIGHT_P1_GPIO_Port, MOTOR_RIGHT_P4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_LEFT_P1_GPIO_Port, MOTOR_LEFT_P4_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOTOR_RIGHT_P4_GPIO_Port, MOTOR_RIGHT_P1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_LEFT_P4_GPIO_Port, MOTOR_LEFT_P1_Pin, GPIO_PIN_SET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(MOTOR_RIGHT_P4_GPIO_Port, MOTOR_RIGHT_P1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_LEFT_P4_GPIO_Port, MOTOR_LEFT_P1_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOTOR_RIGHT_P3_GPIO_Port, MOTOR_RIGHT_P2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_LEFT_P3_GPIO_Port, MOTOR_LEFT_P2_Pin, GPIO_PIN_SET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(MOTOR_RIGHT_P3_GPIO_Port, MOTOR_RIGHT_P2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_LEFT_P3_GPIO_Port, MOTOR_LEFT_P2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOTOR_RIGHT_P2_GPIO_Port, MOTOR_RIGHT_P3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_LEFT_P2_GPIO_Port, MOTOR_LEFT_P3_Pin, GPIO_PIN_SET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(MOTOR_RIGHT_P2_GPIO_Port, MOTOR_RIGHT_P3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_LEFT_P2_GPIO_Port, MOTOR_LEFT_P3_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOTOR_RIGHT_P1_GPIO_Port, MOTOR_RIGHT_P4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_LEFT_P1_GPIO_Port, MOTOR_LEFT_P4_Pin, GPIO_PIN_SET);

	HAL_Delay(1);
}

void rideMotorLeft(void)
{
	  HAL_GPIO_WritePin(MOTOR_RIGHT_P4_GPIO_Port, MOTOR_RIGHT_P4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P4_GPIO_Port, MOTOR_LEFT_P4_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P1_GPIO_Port, MOTOR_RIGHT_P1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P1_GPIO_Port, MOTOR_LEFT_P1_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P1_GPIO_Port, MOTOR_RIGHT_P1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P1_GPIO_Port, MOTOR_LEFT_P1_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P2_GPIO_Port, MOTOR_RIGHT_P2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P2_GPIO_Port, MOTOR_LEFT_P2_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P2_GPIO_Port, MOTOR_RIGHT_P2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P2_GPIO_Port, MOTOR_LEFT_P2_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P3_GPIO_Port, MOTOR_RIGHT_P3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P3_GPIO_Port, MOTOR_LEFT_P3_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P3_GPIO_Port, MOTOR_RIGHT_P3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P3_GPIO_Port, MOTOR_LEFT_P3_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P4_GPIO_Port, MOTOR_RIGHT_P4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P4_GPIO_Port, MOTOR_LEFT_P4_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);
}

void rideMotorRight(void)
{
	  HAL_GPIO_WritePin(MOTOR_RIGHT_P1_GPIO_Port, MOTOR_RIGHT_P1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P1_GPIO_Port, MOTOR_LEFT_P1_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P4_GPIO_Port, MOTOR_RIGHT_P4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P4_GPIO_Port, MOTOR_LEFT_P4_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P4_GPIO_Port, MOTOR_RIGHT_P4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P4_GPIO_Port, MOTOR_LEFT_P4_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P3_GPIO_Port, MOTOR_RIGHT_P3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P3_GPIO_Port, MOTOR_LEFT_P3_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P3_GPIO_Port, MOTOR_RIGHT_P3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P3_GPIO_Port, MOTOR_LEFT_P3_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P2_GPIO_Port, MOTOR_RIGHT_P2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P2_GPIO_Port, MOTOR_LEFT_P2_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P2_GPIO_Port, MOTOR_RIGHT_P2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P2_GPIO_Port, MOTOR_LEFT_P2_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(MOTOR_RIGHT_P1_GPIO_Port, MOTOR_RIGHT_P1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR_LEFT_P1_GPIO_Port, MOTOR_LEFT_P1_Pin, GPIO_PIN_SET);

	  HAL_Delay(1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
//turn
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

