/* USER CODE BEGIN Header */
#pragma anon_unions
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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RHHDriver.h"
#include "queue.h"
#include "string.h"
#include "parser.h"
#include "stdio.h"
#include "stdlib.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ignitionTask */
osThreadId_t ignitionTaskHandle;
const osThreadAttr_t ignitionTask_attributes = {
  .name = "ignitionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for injectionTask */
osThreadId_t injectionTaskHandle;
const osThreadAttr_t injectionTask_attributes = {
  .name = "injectionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readSensorsTask */
osThreadId_t readSensorsTaskHandle;
const osThreadAttr_t readSensorsTask_attributes = {
  .name = "readSensorsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usartTask */
osThreadId_t usartTaskHandle;
const osThreadAttr_t usartTask_attributes = {
  .name = "usartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RHHTask */
osThreadId_t RHHTaskHandle;
const osThreadAttr_t RHHTask_attributes = {
  .name = "RHHTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EEPROMTask */
osThreadId_t EEPROMTaskHandle;
const osThreadAttr_t EEPROMTask_attributes = {
  .name = "EEPROMTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartParserTask */
osThreadId_t uartParserTaskHandle;
const osThreadAttr_t uartParserTask_attributes = {
  .name = "uartParserTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartTXtask */
osThreadId_t uartTXtaskHandle;
const osThreadAttr_t uartTXtask_attributes = {
  .name = "uartTXtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USART_ISR_QUEUE */
osMessageQueueId_t USART_ISR_QUEUEHandle;
const osMessageQueueAttr_t USART_ISR_QUEUE_attributes = {
  .name = "USART_ISR_QUEUE"
};
/* Definitions for USART_TX_QUEUE */
osMessageQueueId_t USART_TX_QUEUEHandle;
const osMessageQueueAttr_t USART_TX_QUEUE_attributes = {
  .name = "USART_TX_QUEUE"
};
/* USER CODE BEGIN PV */
	volatile bool send_info_flag = false;
	char raw_stringRX[80]; //строка, получаемая из UART
	char bufer_for_processingRX[80]; //полученная из юарт строка попадает сюда и идет в обработку
	#define DELIMETER ','
	#define TERMINATOR ';'

	#define EEPROM_WRITE_ADDR 0xA0
	#define EEPROM_READ_ADDR 0xA1
	
volatile bool adc_flag = true;
	//время одного оборота коленвала
	volatile uint32_t turn_around_time = 50000; //в микросекундах
	//карта углов опережения зажигания
	//углы в градусах, умноженные на 10
	float ignition_map[][2] = {{0, 1.0},{1000, 6.5}, {2000, 28.2}, {3000, 39.3}, {4000, 43.9}, {5000, 45.5}, {6000, 45.5}, {7000, 45.5}};
	//float ignition_map[][2] = {{0, 1.0},{1000, 2.2}, {2000, 12.3}, {3000, 21.8}, {4000, 26.3}, {5000, 27.3}, {6000, 27.9}, {7000, 27.5}};
	#define NUM_OF_POINTS_ANGLES 8
	//угол одного из лепестков модулятора, в градусах
	#define MODULATOR_ANGLE 30.0f
	//текущий угол опережения зажигания, в градусах
	volatile float cur_uoz;
	//текущее время задержки подачи искры, в сотнях микросекунд
	volatile uint16_t ign_delay;
	//время после верхней мертвой точки через которое начинается накачка катушки
	volatile uint32_t bobbin_on_delay;
	//время накачки катушки в миллисекундах
	#define COIL_PUMP_TIME 6
	//считаем количество оборотов коленвала на старте
	volatile uint32_t rot_counter = 0;
	
	
	#define AFR 12.0f //соотношение воздуха к топливу 
	volatile float afr = 3;
	//производителность форсунки, грамм/мин, для бензина
	#define NOZZLE_PERFORMANCE 103.5f
	//время открытия форсунки, в сотнях микросекунд
	#define INJECTOR_OPEN_TIME 0
	//время впрыска топлива, в сотнях микросекунд
	volatile uint16_t injecting_time = 0;
	volatile bool inj_flag = false;
	
	volatile uint32_t eng_rot_counter = 0;
	
	//период ПИД регулятора
	#define DELAY_FOR_PID 200
	//обороты, к которым стремится ПИД
	#define TARGET_RPM 1100
	//ограничение снизу
	#define MIN_OUT -4
	//ограничение сверху
	#define MAX_OUT 4
	//коэффиценты ПИД
	#define KP 0.1f
	#define KI 0.01f
	#define KD 0.01f
	//скорость РХХ, шаг/сек
	#define RHH_SPEED 100
	volatile bool rhh_flag = false;
	
	uint8_t tps_axis[16] = {0, 2, 4, 6, 8, 10, 14, 18, 23, 29, 37, 46, 56, 66, 80, 100};
	uint16_t rpm_axis[16] = {600, 800, 1000, 1200, 1600, 2000, 2520, 3000, 3520, 4000, 4520, 5000, 5520, 6000, 7000, 10200};
	float bcf[16][16] = {{143, 141, 142, 145, 148, 148, 150, 153, 156, 159, 162, 164, 166, 168, 169, 170},
											 {138, 140, 143, 148, 149, 152, 156, 159, 162, 164, 166, 168, 169, 170, 171, 172},
											 {128, 149, 149, 150, 165, 172, 178, 178, 180, 178, 178, 178, 178, 178, 178, 178},
											 {129, 135, 152, 148, 169, 227, 216, 218, 208, 204, 199, 196, 194, 191, 189, 188},
											 {104, 121, 138, 150, 182, 217, 251, 267, 247, 238, 229, 221, 214, 209, 205, 201},
											 { 71, 108, 113, 153, 172, 228, 272, 283, 292, 278, 258, 244, 236, 229, 223, 219},
											 { 57,  84,  94, 113, 138, 163, 277, 293, 319, 296, 265, 265, 257, 251, 246, 241},
											 { 57,  67,  83,  99, 107, 150, 201, 258, 299, 299, 283, 285, 282, 280, 275, 267},
											 { 41,  53,  71,  78, 105, 127, 169, 210, 235, 254, 293, 319, 333, 323, 307, 295},
											 { 44,  45,  57,  64,  85, 109, 138, 154, 182, 203, 280, 379, 399, 351, 332, 317},
											 { 44,  44,  48,  54,  63,  77,  96, 116, 137, 164, 256, 388, 347, 346, 331, 320},
											 { 44,  44,  45,  48,  53,  61,  71,  85, 103, 120, 204, 248, 286, 295, 300, 299},
											 { 44,  44,  44,  45,  48,  52,  58,  66,  78,  91, 122, 164, 201, 229, 247, 260},
											 { 44,  44,  44,  44,  45,  47,  51,  56,  63,  73,  87, 110, 139, 167, 192, 209},
											 { 44,  44,  44,  44,  44,  45,  47,  50,  54,  61,  69,  82, 101, 122, 146, 167},
											 { 44,  44,  44,  44,  44,  44,  45,  47,  50,  55,  62,  71,  84, 103, 124, 146}
	};
	float bcf_correction[16][16] = { {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
																	 {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
	};
	
	float lambda_map[][2] = {
														{8.95,   0.8800},
														{9.11,   0.8797},
														{9.26,   0.8792},
														{9.41,   0.8787},
														{9.56,   0.8782},
														{9.71,   0.8778},
														{9.87,   0.8775},
														{10.02,  0.8771},
														{10.17,  0.8757},
														{10.32,  0.8752},
														{10.47,  0.8747},
														{10.63,  0.8742},
														{10.78,  0.8738},
														{10.93,  0.8735},
														{11.08,  0.8731},
														{11.24,  0.8727},
														{11.39,  0.8722},
														{11.54,  0.8717},
														{11.69,  0.8712},
														{11.86,  0.8690},
														{12.04,  0.8520},
														{12.23,  0.8410},
														{12.39,  0.8320},
														{12.62,  0.8180},
														{12.83,  0.8050},
														{13.03,  0.7920},
														{13.21,  0.7790},
														{13.4,   0.7630},
														{13.59,  0.7410},
														{13.82,  0.7180},
														{14.1,   0.6830},
														{14.43,  0.6060},
														{14.83,  0.2310},
														{15.31,  0.1710},
														{15.85,  0.1070},
														{16.47,  0.0630},
														{17.15,  0.0500},
														{17.9,   0.0380},
														{18.7,   0.0330}
};
	
	//Карта датчика температуры охлаждающей жидкости
	//22 точки
	float clt_map[][2] = {{100700, -40}, {52700, -30}, {28680, -20}, {21450, -15}, {16180, -10}, {12300, -5}, {9420, 0}, {7280, 5}, {5670, 10},
		{4450, 15}, {3520, 20}, {2796, 25}, {2238, 30}, {1459, 40}, {1188, 45}, {973, 60}, {667, 65}, {467, 70}, {332, 80}, {241, 90}, {177, 100}, {80, 130}};
	#define NUM_OF_POINTS_CLT 22
	float ctemp = 0;
			
	//карта датчика расхода воздуха
	//float maf_map[][2] = {{0, 0}, {0.3, 10.2}, {0.6, 10.9}, {0.93, 20.8}, {1.55, 45.9}, {1.87, 49.3}, {2.18, 67.6}, {2.5, 91.3}, {2.96, 123}, {3.28, 166}, {3.59, 222}};
	float maf_map[][2] = {{0, 0}, {0.3, 6.2}, {0.6, 9.9}, {0.93, 15.8}, {1.55, 34.9}, {1.87, 49.3}, {2.18, 67.6}, {2.5, 91.3}, {2.96, 123}, {3.28, 166}, {3.59, 222}};
	#define NUM_OF_POINTS_MAF 11
	//потребление воздуха, кг/час
	volatile float consumption_per_hour = 0;
	//расход воздуха на один цикл, грамм
	volatile float air_per_cycle = 0;
	volatile float cur_cf = 0;
	volatile bool air_flag = false;
	volatile uint8_t fuel_calc_mode = 1;	
		
	volatile uint16_t ADC_Data[4];
	volatile uint16_t o2_adc_data = 0;
	volatile uint16_t maf_adc_data = 0;
	volatile uint16_t tps_adc_data = 0;
	volatile uint16_t clt_adc_data = 0;
	
	volatile float o2_volts = 0;
	volatile float maf_volts = 0;
	volatile float tps_volts = 0;
	volatile float clt_volts = 0;
	
	struct engine_parameters{
		uint16_t cur_rpm;
		uint16_t cur_injecting_time;
		float cur_uoz;
		uint16_t cur_ign_delay;
		float cur_temp;
		float cur_afr;
		uint8_t rhh_step;
		float cur_o2;
		uint8_t tps_percent;
	};
	volatile struct engine_parameters eng_param;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartItnitionTask(void *argument);
void StartInjectionTask(void *argument);
void StartReadSensorsTask(void *argument);
void StartUsartTask(void *argument);
void StartRHHTask(void *argument);
void StartEEPROMTask(void *argument);
void StartuartParserTask(void *argument);
void StartUartTXtask(void *argument);

/* USER CODE BEGIN PFP */
void sendInfoToUart(char TXbuf[]);
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
	Delay_100usInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of USART_ISR_QUEUE */
  USART_ISR_QUEUEHandle = osMessageQueueNew (128, sizeof(uint8_t), &USART_ISR_QUEUE_attributes);

  /* creation of USART_TX_QUEUE */
  USART_TX_QUEUEHandle = osMessageQueueNew (512, sizeof(uint8_t), &USART_TX_QUEUE_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ignitionTask */
  ignitionTaskHandle = osThreadNew(StartItnitionTask, NULL, &ignitionTask_attributes);

  /* creation of injectionTask */
  injectionTaskHandle = osThreadNew(StartInjectionTask, NULL, &injectionTask_attributes);

  /* creation of readSensorsTask */
  readSensorsTaskHandle = osThreadNew(StartReadSensorsTask, NULL, &readSensorsTask_attributes);

  /* creation of usartTask */
  usartTaskHandle = osThreadNew(StartUsartTask, NULL, &usartTask_attributes);

  /* creation of RHHTask */
  RHHTaskHandle = osThreadNew(StartRHHTask, NULL, &RHHTask_attributes);

  /* creation of EEPROMTask */
  EEPROMTaskHandle = osThreadNew(StartEEPROMTask, NULL, &EEPROMTask_attributes);

  /* creation of uartParserTask */
  uartParserTaskHandle = osThreadNew(StartuartParserTask, NULL, &uartParserTask_attributes);

  /* creation of uartTXtask */
  uartTXtaskHandle = osThreadNew(StartUartTXtask, NULL, &uartTXtask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 84, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(84000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */
	LL_I2C_Enable(I2C1);
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),14, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 83;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 99;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */
	LL_TIM_EnableIT_UPDATE(TIM3);
	LL_TIM_EnableCounter(TIM3);
  /* USER CODE END TIM3_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM11);

  /* TIM11 interrupt Init */
  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  TIM_InitStruct.Prescaler = 839;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 49999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM11, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM11);
  /* USER CODE BEGIN TIM11_Init 2 */
	LL_TIM_EnableIT_UPDATE(TIM11);
	LL_TIM_EnableCounter(TIM11);
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
	LL_USART_EnableIT_RXNE(USART1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_3
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_8|LL_GPIO_PIN_9);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_3
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE3);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE4);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
		taskYIELD();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartItnitionTask */
void TIM1_TRG_COM_TIM11_IRQHandler()
{
	UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
    if(TIM11->SR & TIM_SR_UIF)           // Проверяем, что это нас переполнение вызывало
    {
        TIM11->SR &= ~TIM_SR_UIF;        //сбросить флаг
				TIM11->CR1 &= ~TIM_CR1_CEN;
				TIM11->CNT = 0;
				ignCoilOff;
    }
		portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );
}



/**
* @brief Function implementing the ignitionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartItnitionTask */
void StartItnitionTask(void *argument)
{
  /* USER CODE BEGIN StartItnitionTask */
	uint8_t stage = 0;
	Delay_100usInit();
	mainReleyOn;
  /* Infinite loop */
  for(;;)
  {
		
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		
		taskENTER_CRITICAL();
		TIM11->CR1 &= ~TIM_CR1_CEN;
		if(TIM11->CNT != 0){ turn_around_time = TIM11->CNT*10; } //Prescaler = 839 -> умножаем на 10. Если оборот коленвала длился более 500 миллисекунд и таймер отключился и сбросился, turn_around_time не меняем
		TIM11->CNT = 0;
		TIM11->CR1 |= TIM_CR1_CEN;
		taskEXIT_CRITICAL();
		
		switch(stage){
			case 0:
				ignCoilOn;
				rot_counter++;
				if(rot_counter > 100){stage = 1;}
				break;
			case 1:
				Delay_100us(ign_delay);
				taskENTER_CRITICAL();
				ignCoilOff;
				taskEXIT_CRITICAL();
				Delay_100us(bobbin_on_delay);
				taskENTER_CRITICAL();
				ignCoilOn;
				taskEXIT_CRITICAL();
				break;
			case 2:
				break;
			default:
				break;
		}
		
		
		inj_flag = true;
		air_flag = true;
		if(!fuelPumpReleyPinState){
			fuelPumpReleyOn;
		}
		
 }
  /* USER CODE END StartItnitionTask */
}

/* USER CODE BEGIN Header_StartInjectionTask */
/**
* @brief Function implementing the injectionTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartInjectionTask */
void StartInjectionTask(void *argument)
{
  /* USER CODE BEGIN StartInjectionTask */
	//fuelPumpReleyOn;
  /* Infinite loop */
  for(;;)
  {
		if(inj_flag == true){
		taskENTER_CRITICAL();
		rightInjectorOn;
		leftInjectorOn;
		taskEXIT_CRITICAL();
		Delay_100us(injecting_time/2);
		taskENTER_CRITICAL();
		rightInjectorOff;
		leftInjectorOff;
		taskEXIT_CRITICAL();
		inj_flag = false;
		}
		taskYIELD();
  }
  /* USER CODE END StartInjectionTask */
}

/* USER CODE BEGIN Header_StartReadSensorsTask */
/**
* @brief Function implementing the readSensorsTask thread.
* @param argument: Not used
* @retval None
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{


}
float ADCToVolts(uint16_t adc){
	return 3.3/4096*adc;
}

float LinInterp(float x, float x1, float x2, float y1, float y2){
	return y1 + ((y2-y1)/(x2-x1))*(x-x1);
}

float MafVoltageToConsumption(float voltage, float map[][2], uint8_t items_number){
	uint8_t i = 0;
	float value;
	for(i = 0; i < items_number; i++){
		if(map[i][0] > voltage){
			break;
		}
	}
	value = (map[i-1][1] + ((map[i][1] - map[i-1][1])/(map[i][0] - map[i-1][0]))*(voltage-map[i-1][0]));
	i = 0;
	return value;
}
float CltVoltageToTemp(float voltage, float map[][2], uint8_t items_number){
	uint8_t i = 0;
	float value;
	float resistance = (5100*voltage)/(3.3f-voltage);
	for(i = 0; i < items_number; i++){
		if(map[i][0] < resistance){
			break;
		}
	}
	value = (map[i-1][1] + ((map[i][1] - map[i-1][1])/(map[i][0] - map[i-1][0]))*(resistance-map[i-1][0]));
	i = 0;
	return value;
}

//вычисление количества воздуха, потребляемого за один цикл
float AirPerCycle(float maf_volts){
	
	consumption_per_hour = MafVoltageToConsumption(maf_volts, maf_map, NUM_OF_POINTS_MAF);
	air_per_cycle = ((float)turn_around_time/(float)10000)*(consumption_per_hour/(float)360); //грамм; делим на 2 и на 100 так как turn_around_time в микросекундах
	return air_per_cycle;
	
}

//вычисление времени, которое открыта форсунка, сотни микросекунд (AFR, количество воздуха за цикл, в граммах)
uint16_t NozzleDelay(float afr, float air_per_cycle){
	return (air_per_cycle*(float)600000)/(afr*NOZZLE_PERFORMANCE) + INJECTOR_OPEN_TIME;
}

uint16_t uSecToRPM(uint32_t turn_around_time){	//возвращает количество оборотов в минуту
	return 60000000 / turn_around_time; 					//turn_around_time - время одного оборота в микросекундах
}

//поиск угла опережения зажигания в массиве, используется линейная интерполяция
float FindAngle(uint32_t turn_around_time, float map[][2], uint8_t items_number) {
	uint8_t i = 0;
	float angle;
	float rpm = (float)60000000 / (float)turn_around_time;

	for (i = 0; i < items_number; i++) {
		if (map[i][0] > rpm) {
			break;
		}
	}
	angle = (map[i - 1][1] + ((map[i][1] - map[i - 1][1]) / (map[i][0] - map[i - 1][0])) * (rpm - map[i - 1][0]));
	return angle;
}

//вычисление задержки для отключения катушки, в сотнях микросекунд
uint16_t FindIgnDelay(uint32_t turn_around_time, float angle, float modulator_angle) {
	uint16_t delay;
	delay = (float)turn_around_time * (modulator_angle * 2.0f - angle) / 360.0f;

	return delay / 100;
}

/* USER CODE END Header_StartReadSensorsTask */
void StartReadSensorsTask(void *argument)
{
  /* USER CODE BEGIN StartReadSensorsTask */
	uint8_t temp;
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &ADC_Data,4);
	volatile float air_sum = 0;
	volatile uint32_t air_counter = 0;
	uint8_t rpm_axis_coord = 0;
	uint8_t tps_axis_coord = 0;
  /* Infinite loop */
  for(;;)
  {
		
		osDelay(5);
		o2_volts = ADCToVolts(ADC_Data[0]);
		maf_volts = ADCToVolts(ADC_Data[1]);
		air_sum+= maf_volts;
		air_counter++;
		tps_volts = ADCToVolts(ADC_Data[2]);
		clt_volts = ADCToVolts(ADC_Data[3]);
		
		switch(fuel_calc_mode){
			case 0:
				if(air_flag == true){
					injecting_time = NozzleDelay(afr, AirPerCycle(air_sum/(float)air_counter*1.51f));
					if(rot_counter < 50){
						injecting_time = 100;
					}
					if(injecting_time < 30){
						injecting_time = 30;
					}
				air_counter = 0;
				air_sum = 0;
				air_flag = false;
				}
				break;
			case 1:
				injecting_time = LinInterp(tps_volts, 0.05, 0.33, 35, 100)/afr*14.7f;
				break;
			case 2:
				for(uint8_t i = 0; i<16; i++){
					if(eng_param.tps_percent > tps_axis[i]){ 
						rpm_axis_coord = i;
						break;
					}
				}
				for(uint8_t i = 0; i<16; i++){
					if(eng_param.cur_rpm > rpm_axis[i]){ 
						tps_axis_coord = i;
						break;
					}
				}
				air_per_cycle = bcf[rpm_axis_coord][tps_axis_coord];
				cur_cf = bcf[rpm_axis_coord][tps_axis_coord];
				injecting_time = NozzleDelay(afr, cur_cf/1000.0f);
				if(rot_counter < 20){
						injecting_time = 150;
					}
				break;
			}
		
		ctemp = CltVoltageToTemp(clt_volts, clt_map, NUM_OF_POINTS_CLT);
		
		
			if(ctemp < 15){afr = 4.0;}
			else if(ctemp < 20){afr = 6.0;}
			else if(ctemp < 30){afr = 8.0;}
			else if(ctemp < 40){afr = 9.0;}
			else if(ctemp < 50){afr = 10.0;}
			else if(ctemp < 60){afr = 11.0;}
			else if(ctemp < 70){afr = 12.0;}
			else if(ctemp < 80){afr = 12.0;}
			else {afr = 12.0;}
		
		cur_uoz = FindAngle(turn_around_time, ignition_map, NUM_OF_POINTS_ANGLES);
		ign_delay = FindIgnDelay(turn_around_time, cur_uoz, MODULATOR_ANGLE);
		bobbin_on_delay = (turn_around_time - (COIL_PUMP_TIME*1000) - (ign_delay*100))/100;
		
		eng_param.tps_percent = LinInterp(tps_volts, 0.05, 0.33, 0, 100);
		eng_param.cur_afr = afr;
		eng_param.cur_rpm = uSecToRPM(turn_around_time);
		eng_param.cur_temp = ctemp;
		eng_param.cur_injecting_time = injecting_time;
		eng_param.cur_uoz = cur_uoz;
		eng_param.cur_ign_delay = ign_delay;
		eng_param.cur_o2 = o2_volts;
		
		taskYIELD();
  }
  /* USER CODE END StartReadSensorsTask */
}

/* USER CODE BEGIN Header_StartUsartTask */
/**
* @brief Function implementing the usartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTask */
void StartUsartTask(void *argument)
{
  /* USER CODE BEGIN StartUsartTask */
	uint16_t counter = 0;
	char c;
	
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(USART_ISR_QUEUEHandle, &c, portMAX_DELAY);
		raw_stringRX[counter] = c;
		counter++;
		if(c == TERMINATOR){
			strcpy(bufer_for_processingRX, raw_stringRX);
			counter = 0;
			memset(raw_stringRX, 0, sizeof(raw_stringRX));
			xQueueReset(USART_ISR_QUEUEHandle);
			xTaskNotifyGive(uartParserTaskHandle);
		}
		
  }
  /* USER CODE END StartUsartTask */
}

/* USER CODE BEGIN Header_StartRHHTask */
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = integral + (float)err * dt * ki;
  if(integral < minOut){integral = minOut;}
  else if(integral > maxOut){integral = maxOut;}
  
  float D = (err - prevErr) / dt;
  prevErr = err;
  
  int out = err * kp + integral + D * kd;
  if(out < minOut){out = minOut;}
  else if(out > maxOut){out = maxOut;}
  return out;
}
/**
* @brief Function implementing the RHHTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRHHTask */
void StartRHHTask(void *argument)
{
  /* USER CODE BEGIN StartRHHTask */
  /* Infinite loop */
	stepRHH(1, 250, RHH_SPEED);
	stepRHH(0, 30, RHH_SPEED);
	eng_param.rhh_step = 30;
	volatile int out;
  for(;;)
  {
		osDelay(100);
		if((rhh_flag == true) && (rot_counter > 50)){
		if(tps_volts < 0.07f){
			out = computePID((float)eng_param.cur_rpm, TARGET_RPM, KP, KI, KD, (float)DELAY_FOR_PID/1000.0f , MIN_OUT, MAX_OUT);
			if(out > 0){
				if((eng_param.rhh_step + out) < 250){
					stepRHH(0, out, RHH_SPEED);
					eng_param.rhh_step += out;
				}
			}else if(out < 0){
				if(abs(out) < eng_param.rhh_step){
					stepRHH(1, abs(out), RHH_SPEED);
					eng_param.rhh_step -= abs(out);
				}else if(abs(out) > eng_param.rhh_step){
					stepRHH(1, eng_param.rhh_step, RHH_SPEED);
					eng_param.rhh_step = 0;
				}
			}
		}
	}
  }
  /* USER CODE END StartRHHTask */
}

/* USER CODE BEGIN Header_StartEEPROMTask */
void EEPROMWriteByte(I2C_TypeDef *I2Cx, uint8_t address, uint16_t mem_address, uint8_t data){
	
	//старт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
	
	//отправка старшего байта адреса памяти
	I2Cx->DR = (mem_address >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	//отправка младшего байта адреса памяти
	I2Cx->DR = (mem_address & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
		
	//пишем данные	
	I2Cx->DR = data;	
	while(!(I2Cx->SR1 & I2C_SR1_BTF)){};	
	I2Cx->CR1 |= I2C_CR1_STOP;
}

void EEPROMWriteUint16(I2C_TypeDef *I2Cx, uint8_t address, uint16_t mem_address, uint16_t data){
	
	//старт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
	
	//отправка старшего байта адреса памяти
	I2Cx->DR = (mem_address >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	//отправка младшего байта адреса памяти
	I2Cx->DR = (mem_address & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
		
	//пишем данные	
	I2Cx->DR = (data >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};	
	I2Cx->DR = (data & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_BTF)){};	
	I2Cx->CR1 |= I2C_CR1_STOP;
}
uint16_t EEPROMRandReadUint16(I2C_TypeDef *I2Cx, uint8_t address_write, uint8_t address_read, uint16_t mem_address){
	uint8_t high;
	uint8_t low;
	
	//старт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address_write;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
	
	//отправка старшего байта адреса памяти
	I2Cx->DR = (mem_address >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	//отправка младшего байта адреса памяти
	I2Cx->DR = (mem_address & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	
	//рестарт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
	
	//отправка адреса слейва
	I2Cx->DR = address_read;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
		
	//читаем данные	
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)){};
	high = I2Cx->DR;
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)){};
	low = I2Cx->DR;
	I2Cx->CR1 |= I2C_CR1_STOP;
		
	return (high << 8) | low;
}


void EEPROMWriteFloat(I2C_TypeDef *I2Cx, uint8_t address, uint16_t mem_address, float data){
	typedef union _FloType
{
    struct
			{
        uint8_t low2;
				uint8_t low1;
				uint8_t high2;
        uint8_t high1;
    };
    float flo;
}FloType;

FloType flo;
flo.flo = data;
	//старт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
	
	//отправка старшего байта адреса памяти
	I2Cx->DR = (mem_address >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	//отправка младшего байта адреса памяти
	I2Cx->DR = (mem_address & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
		
	//пишем данные	
	I2Cx->DR = (flo.high1);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	I2Cx->DR = (flo.high2);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	I2Cx->DR = (flo.low1);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){}		
	I2Cx->DR = (flo.low2);	
	while(!(I2Cx->SR1 & I2C_SR1_BTF)){};	
	I2Cx->CR1 |= I2C_CR1_STOP;
}
float EEPROMRandReadFloat(I2C_TypeDef *I2Cx, uint8_t address_write, uint8_t address_read, uint16_t mem_address){
	typedef union _FloType
{
    struct
    {
        uint8_t low2;
				uint8_t low1;
				uint8_t high2;
        uint8_t high1;
    };
    float flo;
}FloType;

FloType flo;

	//старт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address_write;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
	
	//отправка старшего байта адреса памяти
	I2Cx->DR = (mem_address >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	//отправка младшего байта адреса памяти
	I2Cx->DR = (mem_address & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	
	//рестарт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address_read;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
		
	//читаем	
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)){};
	flo.high1 = I2Cx->DR;
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)){};
	flo.high2 = I2Cx->DR;	
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)){};
	flo.low1 = I2Cx->DR;
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)){};
	flo.low2 = I2Cx->DR;
	I2Cx->CR1 |= I2C_CR1_STOP;
	
	return flo.flo;
}



void EEPROMWritePage(I2C_TypeDef *I2Cx, uint8_t address, uint16_t mem_address, uint8_t* data){
	
	uint8_t length = sizeof(data);
	
	//старт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
	
	//отправка старшего байта адреса памяти
	I2Cx->DR = (mem_address >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	//отправка младшего байта адреса памяти
	I2Cx->DR = (mem_address & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	
	//пишем данные
	for(uint8_t i = 0; i < (length-1); i++){
		I2Cx->DR = data[i];	
		while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	}	
	I2Cx->DR = data[length-1];	
	while(!(I2Cx->SR1 & I2C_SR1_BTF)){};	
	I2Cx->CR1 |= I2C_CR1_STOP;
}
void EEPROMReadPage(I2C_TypeDef *I2Cx, uint8_t address_write, uint8_t address_read, uint16_t mem_address, uint8_t* data, uint8_t length){
	
	//старт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address_write;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
	
	//отправка старшего байта адреса памяти
	I2Cx->DR = (mem_address >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	//отправка младшего байта адреса памяти
	I2Cx->DR = (mem_address & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	
	//рестарт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address_read;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
		
	//пишем данные
	for(uint8_t i = 0; i < (length); i++){
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)){};
	data[i] = I2Cx->DR;
	}	
	I2Cx->CR1 |= I2C_CR1_STOP;
}
uint8_t EEPROMRandReadByte(I2C_TypeDef *I2Cx, uint8_t address_write, uint8_t address_read, uint16_t mem_address){
	uint8_t data;
	
	//старт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;
		
	//отправка адреса слейва
	I2Cx->DR = address_write;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
	
	//отправка старшего байта адреса памяти
	I2Cx->DR = (mem_address >> 8);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
	//отправка младшего байта адреса памяти
	I2Cx->DR = (mem_address & 0xff);	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};
		
	//рестарт
	I2Cx->CR1 |= I2C_CR1_START;
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void)I2Cx->SR1;	
		
	//отправка адреса слейва
	I2Cx->DR = address_read;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
		
		
	//читаем	
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)){};
	data = I2Cx->DR;	
	I2Cx->CR1 |= I2C_CR1_STOP;
	return data;
}
/**
* @brief Function implementing the EEPROMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEEPROMTask */
void StartEEPROMTask(void *argument)
{
  /* USER CODE BEGIN StartEEPROMTask */
	char TXbuf[128];
  /* Infinite loop */
  for(;;)
  {
		if(send_info_flag == false){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		send_info_flag = true;
		}
		
		osDelay(pdMS_TO_TICKS(200));
		taskENTER_CRITICAL();
		sprintf(TXbuf, "%c,%d,%f,%f,%d,%d,%d,%f,%f%c", 'd', eng_param.cur_rpm, eng_param.cur_temp, eng_param.cur_uoz, eng_param.cur_ign_delay, eng_param.cur_injecting_time, eng_param.rhh_step, eng_param.cur_o2, eng_param.cur_afr, ';');
		sendInfoToUart(TXbuf);
		sprintf(TXbuf, "%s,%f,%f,%f,%f,%f%c", "d2", o2_volts, clt_volts, tps_volts, maf_volts, air_per_cycle, ';');
		sendInfoToUart(TXbuf);
		taskEXIT_CRITICAL();
		
  }
  /* USER CODE END StartEEPROMTask */
}

/* USER CODE BEGIN Header_StartuartParserTask */
/**
* @brief Function implementing the uartParserTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartuartParserTask */
void StartuartParserTask(void *argument)
{
  /* USER CODE BEGIN StartuartParserTask */
	char TXbuf[128];
	uint32_t* package;
	uint16_t delay;
	uint8_t data;
	uint8_t array[12];
  /* Infinite loop */
  for(;;)
  {
		package = parseString(bufer_for_processingRX, DELIMETER, TERMINATOR);
		memset(bufer_for_processingRX, 0, sizeof(bufer_for_processingRX));
		
		switch(package[0]){
						case 0:
							
							break;
						case 1:
							if(package[1] == 1){
								mainReleyOn;
								sprintf(TXbuf, "%s", "mainReleyOn;");
							}else if(package[1] == 0){
								mainReleyOff;
								sprintf(TXbuf, "%s", "mainReleyOff;");
							}
							sendInfoToUart(TXbuf);
							break;
						case 2:
							if(package[1] == 1){
								fuelPumpReleyOn;
								sprintf(TXbuf, "%s", "fuelPumpReleyOn;");
							}else if(package[1] == 0){
								fuelPumpReleyOff;
								sprintf(TXbuf, "%s", "fuelPumpReleyOff;");
							}
							sendInfoToUart(TXbuf);
							break;
						case 3:
							if(package[1] == 1){
							xTaskNotifyGive(EEPROMTaskHandle);
							}else if(package[1] == 0){
								send_info_flag = false;
							}
							break;
						case 4:
							if(package[1] == 1){
								rhh_flag = true;
							}else{
								rhh_flag = false;
							}
							break;
						case 5:
							sprintf(TXbuf, "%s %d %s %d %s %d %c", "dir", package[1], "count", package[2], "speed", package[3],';');
							sendInfoToUart(TXbuf);
							stepRHH(package[1], package[2], package[3]);
							break;
						case 6:
							afr = (float)package[1];
							sprintf(TXbuf, "%s %f %c", "afr: ", afr,';');
							sendInfoToUart(TXbuf);
							break;
						case 7:
							sprintf(TXbuf, "%s %d %c", "rpm ", eng_param.cur_rpm, ';');
							sendInfoToUart(TXbuf);
							break;
						case 8:
							if(package[1] == 1){
								LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_13);
								sprintf(TXbuf, "%s %c", "led OFF", ';');
							}else if(package[1] == 0){
								LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_13);
								sprintf(TXbuf, "%s %c", "led ON", ';');
							}
							sendInfoToUart(TXbuf);
							break;
						case 9:
							delay = package[1];
							ignCoilOn;
							Delay_100us(delay/100);
							ignCoilOff;
							break;
						case 10:
							EEPROMWriteByte(I2C1, EEPROM_WRITE_ADDR, 0x0000, package[1]);
							sprintf(TXbuf, "%s", "Ok;");
							sendInfoToUart(TXbuf);
							break;
						case 11:
							data = EEPROMRandReadByte(I2C1, EEPROM_WRITE_ADDR, EEPROM_READ_ADDR, 0x0000);
						sprintf(TXbuf, "%s%d%c", "Data:", data ,';');
							sendInfoToUart(TXbuf);
							break;
						case 12:
							EEPROMReadPage(I2C1, EEPROM_WRITE_ADDR, EEPROM_READ_ADDR, 0x0000, array, 12); 
						sprintf(TXbuf, "%s%d%c", "Data:", data ,';');
							sendInfoToUart(TXbuf);
							break;
						default:
							sprintf(TXbuf, "%s", "Invalid command!;");
							sendInfoToUart(TXbuf);
							break;
					}
		memset(TXbuf, 0, sizeof(TXbuf));
		memset(package, 0, 20);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  }
  /* USER CODE END StartuartParserTask */
}

/* USER CODE BEGIN Header_StartUartTXtask */

void sendInfoToUart(char *TXbuf){
	for(uint8_t i = 0; i < strlen(TXbuf); i++){
		if(TXbuf[i] != TERMINATOR){
			xQueueSendToBack(USART_TX_QUEUEHandle, &TXbuf[i], portMAX_DELAY);
		}else{
			xQueueSendToBack(USART_TX_QUEUEHandle, &TXbuf[i], portMAX_DELAY);
			xTaskNotifyGive(uartTXtaskHandle);
			break;
		}
	}

}
/**
* @brief Function implementing the uartTXtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTXtask */
void StartUartTXtask(void *argument)
{
  /* USER CODE BEGIN StartUartTXtask */
	char c;
  /* Infinite loop */
  for(;;)
  {
		while(xQueueReceive(USART_TX_QUEUEHandle, &c, pdMS_TO_TICKS(2))){
			while(!LL_USART_IsActiveFlag_TXE(USART1)){}
			taskENTER_CRITICAL();
			LL_USART_TransmitData8(USART1, c);
			taskEXIT_CRITICAL();
		}
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  /* USER CODE END StartUartTXtask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
