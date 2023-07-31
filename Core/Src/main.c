/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx.h" // Replace with your STM32 header file

#include "bsp.h"
#include "xmodem.h"
#include "xmodem_global_handle.h"
//#include "flash.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOOTLOADER_START_ADDRESS    0x08000000 // Bootloader start address in flash
#define APPLICATION_START_ADDRESS   0x08008000 // Application start address in flash
#define FLASH_SECTOR_SIZE           0x10000    // Flash sector size (64 KB)
#define APPLICATION_RESET_VECTOR    0x00000004 // Reset vector address in application
#define STARTUP_STRING              "START"    // Startup string to trigger bootloader
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart5;
static uint8_t __PageWritePosition = 0;
static uint32_t __ComTimeoutStartTime, __ComTimeoutLength;
static uint8_t __bootloaderPrompt[] = "\r\nBootloader v1.0\r\n";
static uint8_t __response[] = "*";
static uint8_t __rxBuffer[32];
static uint8_t __rxBufferPosition = 0;
static uint8_t __password[8] = "password";

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI4_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef void (*AppEntryFunc)(void);
typedef void (application_t)(void);
typedef struct
{
    uint32_t		stack_addr;     // Stack Pointer
    application_t*	func_p;        // Program Counter
} JumpStruct;

typedef struct
{
	uint32_t SectorNumber;
	uint32_t SectorBaseAddress;
	uint32_t SectorLength;
} SectorDefinition_t;

uint32_t __InternalFlashAddress = 0x08008000;

static const SectorDefinition_t __Sectors[] = {
  { FLASH_SECTOR_0,  0x08000000, 0x0004000},
  { FLASH_SECTOR_1,  0x08004000, 0x0004000},
  { FLASH_SECTOR_2,  0x08008000, 0x0004000},
  { FLASH_SECTOR_3,  0x0800C000, 0x0004000},
  { FLASH_SECTOR_4,  0x08010000, 0x0010000},
  { FLASH_SECTOR_5,  0x08020000, 0x0020000},
  { FLASH_SECTOR_6,  0x08040000, 0x0020000},
  { FLASH_SECTOR_7,  0x08060000, 0x0020000},
  { FLASH_SECTOR_8,  0x08080000, 0x0020000},
  { FLASH_SECTOR_9,  0x080A0000, 0x0020000},
  { FLASH_SECTOR_10, 0x080C0000, 0x0020000},
  { FLASH_SECTOR_11, 0x080E0000, 0x0020000},
  { FLASH_SECTOR_12, 0x08100000, 0x0004000},
  { FLASH_SECTOR_13, 0x08104000, 0x0004000},
  { FLASH_SECTOR_14, 0x08108000, 0x0004000},
  { FLASH_SECTOR_15, 0x0810C000, 0x0004000},
  { FLASH_SECTOR_16, 0x08110000, 0x0010000},
  { FLASH_SECTOR_17, 0x08120000, 0x0020000},
  { FLASH_SECTOR_18, 0x08140000, 0x0020000},
  { FLASH_SECTOR_19, 0x08160000, 0x0020000},
  { FLASH_SECTOR_20, 0x08180000, 0x0020000},
  { FLASH_SECTOR_21, 0x081A0000, 0x0020000},
  { FLASH_SECTOR_22, 0x081C0000, 0x0020000},
  { FLASH_SECTOR_23, 0x081E0000, 0x0020000}
};

void JumpToApplication(void)
{
	const JumpStruct* vector_p = (JumpStruct*)APPLICATION_START_ADDRESS;

    // Reset the application-specific hardware peripherals
    // Reset UART5
    __HAL_RCC_UART5_FORCE_RESET();
    __HAL_RCC_UART5_RELEASE_RESET();

    // Reset SPI4
    __HAL_RCC_SPI4_FORCE_RESET();
    __HAL_RCC_SPI4_RELEASE_RESET();

    // Reset GPIOD
    __HAL_RCC_GPIOD_FORCE_RESET();
    __HAL_RCC_GPIOD_RELEASE_RESET();

    // Reset GPIOC
    __HAL_RCC_GPIOC_FORCE_RESET();
    __HAL_RCC_GPIOC_RELEASE_RESET();

//    __HAL_RCC_UART5_CLK_DISABLE();
//    __HAL_RCC_SPI4_CLK_DISABLE();
//    __HAL_RCC_GPIOC_CLK_DISABLE();
//    __HAL_RCC_GPIOD_CLK_DISABLE();

    HAL_RCC_DeInit();
    HAL_DeInit();

	/* let's do The Jump! */
    /* Jump, used asm to avoid stack optimization */
    asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));
}

// Function to erase a sector of the flash
void Flash_EraseSector(uint32_t sector)
{
	uint32_t lame;
//	HAL_FLASH_OB_Unlock();             // Unlock the option bytes for write access
	FLASH_OBProgramInitTypeDef obInit; // Option bytes structure
	HAL_FLASHEx_OBGetConfig(&obInit);   // Get the current option bytes configuration
//
//	// Disable write protection for the sector
//	obInit.WRPSector &= ~(sector); // Replace x with the sector number
//	obInit.OptionType = OPTIONBYTE_WRP;
//	if (HAL_FLASHEx_OBProgram(&obInit) != HAL_OK)
//	{
//	    // Option bytes programming failed
//	    // Handle the error here
//	}
//
////	HAL_FLASH_OB_Lock(); // Lock the option bytes after modification

	HAL_StatusTypeDef status;
	status = HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Banks = 0;
    eraseInit.Sector = sector;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    status = HAL_FLASHEx_Erase(&eraseInit, &lame);
    if( status != HAL_OK )
    {
    	while(1);
    }
}

// Function to write data to the flash
void Flash_WriteData(uint32_t address, uint8_t* data, uint32_t length)
{
	HAL_StatusTypeDef status;
	status = HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < length; i += 4)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, *(uint32_t*)(data + i));
        if( status != HAL_OK )
        {
        	while(1);
        }
    }
    status = HAL_FLASH_Lock();
}


static void __TimerStart( uint32_t timeout )
{
  __ComTimeoutStartTime = BspSystemTimeGetTime();
  __ComTimeoutLength    = timeout;
}

static void __TimerKill( void )
{

}

static uint8_t __TimerExpired( void )
{
  return BspSystemTimeHasExpired( __ComTimeoutStartTime, __ComTimeoutLength );
}

static void __ComWriteSlaveMode( uint8_t Byte )
{
	UART_SendByte( Byte, 100 );
}

static uint8_t __ComReadSlaveMode( uint8_t * Byte )
{
	return UART_ReceiveByte( Byte );
}

static uint8_t __ComPayloadCallbackSlaveMode( uint8_t * Payload )
{
  // Flashing Data, Size of data is in words
  Flash_WriteData(__InternalFlashAddress, Payload, 1024);

  // Updating Address with 1K Bytes
  __InternalFlashAddress += 1024;

  return 1;
}


static void __PrepareInternalFlash( void )
{
	uint8_t sector = 0;
	for( sector = FLASH_SECTOR_2; sector <= FLASH_SECTOR_19; sector++ )
	{
		Flash_EraseSector(sector);
	}
}

static void __resetRxBuffer(void)
{
	memset(__rxBuffer, 0, sizeof(__rxBuffer) );
	__rxBufferPosition = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int strCmpResult = 0;
  uint8_t __waitForSecret = 1;
  uint8_t __rxByte = 0;
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
  MX_UART5_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  __resetRxBuffer();
  BspSystemTimeInit();
  XModemGlobalHandle  = XModem1KCreate( __ComWriteSlaveMode,
		  	  	  	  	  	  	  	  	__ComReadSlaveMode,
										__TimerStart,
										__TimerKill,
										__TimerExpired      );

  UART_SendData( __bootloaderPrompt, strlen((char*)__bootloaderPrompt), 1000 );

  // I should be able to simply reuse this timer before XModem1K Starts
  __TimerStart(3000);
  while( __waitForSecret )
  {
	  if( UART_ReceiveByte( &__rxByte ) )
	  {
		  __rxBuffer[__rxBufferPosition++] = __rxByte;
	  }
	  if( __TimerExpired() || ( __rxBufferPosition == 8 ) )
	  {
		  strCmpResult = strncmp( (char*)__rxBuffer, (char*)__password, sizeof(__password));
		  __waitForSecret = 0;
	  }
  }

  UART_SendData( __bootloaderPrompt, strlen((char*)__bootloaderPrompt), 1000 );

  if( 0 == strCmpResult )
  {
	  UART_SendData( __response, strlen((char*)__response), 1000 );
	  __PrepareInternalFlash();
	  XModem1KReceiveFileInit( XModemGlobalHandle );
	  XModem1KReceiveFile( XModemGlobalHandle, __ComPayloadCallbackSlaveMode );
	  HAL_FLASH_Lock();

	  // TODO: We probably need to add the FLASH_LOCK() to an error handler somewhere
  }

  JumpToApplication();

  // TODO: Rip out the unnecessary FreeRTOS stuff
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

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
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
