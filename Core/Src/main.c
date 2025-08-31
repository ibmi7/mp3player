/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include <stdio.h>
#include "SDCard.h"
#include "ff.h"
#include "portmacro.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STREAM_BUFFER_NUM 4
#define STREAM_BUFFER_SIZE 4096 //use 4KB for buffer
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId GUI_TaskHandle;
osThreadId Audio_TaskHandle;
osThreadId Mp3Decoder_TaskHandle;
osThreadId SdCard_TaskHandle;
osThreadId Playback_TaskHandle;
osThreadId UART_TaskHandle;
osMessageQId sdQueueHandle;
osMessageQId uartQueueHandle;
osMessageQId freeBufferQueueHandle;
osMessageQId filledBufferQueueHandle;
/* USER CODE BEGIN PV */
static uint8_t sdBuffers[STREAM_BUFFER_NUM][STREAM_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void GUI_Start(void const * argument);
void Audio_Start(void const * argument);
void Decoder_Start(void const * argument);
void SD_Start(void const * argument);
void Playback_Start(void const * argument);
void UART_Start(void const * argument);

/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  char *msg = pvPortMalloc(strlen(buffer)+1);
  strcpy(msg, buffer);
  xQueueSend(uartQueueHandle, &msg, portMAX_DELAY);

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of sdQueue */
  osMessageQDef(sdQueue, 16, SD_Job_t*);
  sdQueueHandle = osMessageCreate(osMessageQ(sdQueue), NULL);

  /* definition and creation of uartQueue */
  osMessageQDef(uartQueue, 16, char*);
  uartQueueHandle = osMessageCreate(osMessageQ(uartQueue), NULL);

  /* definition and creation of freeBufferQueue */
  osMessageQDef(freeBufferQueue, 4, void*);
  freeBufferQueueHandle = osMessageCreate(osMessageQ(freeBufferQueue), NULL);

  /* definition and creation of filledBufferQueue */
  osMessageQDef(filledBufferQueue, 4, void*);
  filledBufferQueueHandle = osMessageCreate(osMessageQ(filledBufferQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GUI_Task */
  osThreadDef(GUI_Task, GUI_Start, osPriorityBelowNormal, 0, 1024);
  GUI_TaskHandle = osThreadCreate(osThread(GUI_Task), NULL);

  /* definition and creation of Audio_Task */
  osThreadDef(Audio_Task, Audio_Start, osPriorityRealtime, 0, 256);
  Audio_TaskHandle = osThreadCreate(osThread(Audio_Task), NULL);

  /* definition and creation of Mp3Decoder_Task */
  osThreadDef(Mp3Decoder_Task, Decoder_Start, osPriorityHigh, 0, 1024);
  Mp3Decoder_TaskHandle = osThreadCreate(osThread(Mp3Decoder_Task), NULL);

  /* definition and creation of SdCard_Task */
  osThreadDef(SdCard_Task, SD_Start, osPriorityAboveNormal, 0, 512);
  SdCard_TaskHandle = osThreadCreate(osThread(SdCard_Task), NULL);

  /* definition and creation of Playback_Task */
  osThreadDef(Playback_Task, Playback_Start, osPriorityNormal, 0, 256);
  Playback_TaskHandle = osThreadCreate(osThread(Playback_Task), NULL);

  /* definition and creation of UART_Task */
  osThreadDef(UART_Task, UART_Start, osPriorityLow, 0, 512);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SPI_CS_GPIO_Port, SD_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_SPI_CS_Pin */
  GPIO_InitStruct.Pin = SD_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
    // create sd job
    static SD_Job_t job;
    job.type = SD_JOB_LISTDIRECTORY;
    SD_SubmitJob(&job, sdQueueHandle);
    for(;;)
    {
        osDelay(1);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GUI_Start */
/**
* @brief Function implementing the GUI_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GUI_Start */
void GUI_Start(void const * argument)
{
  /* USER CODE BEGIN GUI_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GUI_Start */
}

/* USER CODE BEGIN Header_Audio_Start */
/**
* @brief Function implementing the Audio_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Audio_Start */
void Audio_Start(void const * argument)
{
  /* USER CODE BEGIN Audio_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Audio_Start */
}

/* USER CODE BEGIN Header_Decoder_Start */
/**
* @brief Function implementing the Mp3Decoder_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Decoder_Start */
void Decoder_Start(void const * argument)
{
  /* USER CODE BEGIN Decoder_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Decoder_Start */
}

/* USER CODE BEGIN Header_SD_Start */
/**
* @brief Function implementing the SdCard_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SD_Start */
void SD_Start(void const * argument)
{
  /* USER CODE BEGIN SD_Start */
  /* Infinite loop */
    SD_Job_t *currentJob;
    FIL file;
    FRESULT res;
    void* stream_buffer;
    osDelay(1000);
    /* Initialize buffers */
    for (int i = 0; i < STREAM_BUFFER_NUM; i++) {
      void *ptr = sdBuffers[i];
      xQueueSend(freeBufferQueueHandle, &ptr, 0);
    }
    /* Initialie SD Card*/
    res = SD_Initialize();
    if (res != FR_OK)
    {
        myprintf("Failed to initialize SD Card with error %d\n", res);
        while(1);
    }
    myprintf("SD Initialized.\n");
    for(;;)
    {
        /* State Machine handling*/
        if (xQueueReceive(sdQueueHandle, &currentJob, portMAX_DELAY) == pdPASS)
        {
            switch (currentJob->type) {
                case SD_JOB_CREATE:
                {
                    res = f_open(&file, currentJob->filename, FA_CREATE_NEW | FA_WRITE);
                    if (res == FR_OK) f_close(&file);
                    currentJob->status = (res == FR_OK || res == FR_EXIST) ? SD_JOB_SUCCESS : SD_JOB_FAILED;
                    currentJob->fresult = res;
                    break;
                }
                case SD_JOB_READ:
                {
                    res = f_open(&file, currentJob->filename, FA_READ);
                    if ( res == FR_OK) res = f_read(&file, currentJob->buffer, currentJob->length, &(currentJob->bytesTransferred));
                    f_close(&file);
                    currentJob->status = (res == FR_OK) ? SD_JOB_SUCCESS : SD_JOB_FAILED;
                    currentJob->fresult = res;
                    break;
                }
                case SD_JOB_WRITE:
                {
                    res = f_open(&file, currentJob->filename, FA_WRITE | FA_OPEN_ALWAYS);
                    if (res == FR_OK) res = f_lseek(&file, f_size(&file));  // Move to EOF to append
                    if (res == FR_OK) res = f_write(&file, currentJob->buffer, currentJob->length, &(currentJob->bytesTransferred));
                    f_close(&file);
                    currentJob->status = (res == FR_OK) ? SD_JOB_SUCCESS : SD_JOB_FAILED;
                    currentJob->fresult = res;
                    break;
                }
                case SD_JOB_GETINFO:
                {
                    res = SD_GetInfo(&(currentJob->info->totalMB), &(currentJob->info->freeMB));
                    currentJob->status = (res == FR_OK) ? SD_JOB_SUCCESS : SD_JOB_FAILED;
                    currentJob->fresult = res;
                    break;
                    
                }
                case SD_JOB_FORMAT:
                {
                    BYTE work[_MAX_SS];
                    res = f_mkfs("", FM_ANY, 0, work, sizeof(work));
                    currentJob->status = (res == FR_OK) ? SD_JOB_SUCCESS : SD_JOB_FAILED;
                    currentJob->fresult = res;
                    break;
                }
                case SD_JOB_LISTDIRECTORY:
                {
                    DIR dir;
                    FILINFO fno;
                    FRESULT res;
                    res = f_opendir(&dir, currentJob->filename);
                    if ( res != FR_OK) {
                        myprintf("Failed to open directory.\n");
                    }
                    else
                    {
                      while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
                          // Format output: add file/dir type, size, etc. as needed
                          myprintf("%s%s\n", (fno.fattrib & AM_DIR) ? "[DIR] " : "      ", fno.fname);
                      }
                      res = f_closedir(&dir);
                    }
                    currentJob->status = (res == FR_OK) ? SD_JOB_SUCCESS : SD_JOB_FAILED;
                    currentJob->fresult = res;

                    break;
                }
                case SD_JOB_DELETE:
                {
                    res = f_unlink(currentJob->filename);
                    currentJob->status = (res == FR_OK) ? SD_JOB_SUCCESS : SD_JOB_FAILED;
                    currentJob->fresult = res;
                    break;
                }
                case SD_JOB_STREAM:
                {
                  res = f_open(&file, currentJob->filename, FA_READ);
                  for (;;){
                    if (xQueueReceive(freeBufferQueueHandle, &stream_buffer, portMAX_DELAY)){
                      f_read(&file, stream_buffer, STREAM_BUFFER_SIZE, &(currentJob->bytesTransferred));
                      xQueueSend(filledBufferQueueHandle, &stream_buffer, portMAX_DELAY);

                      if (currentJob->bytesTransferred < STREAM_BUFFER_SIZE){
                        // EOF reached, job done
                        currentJob->status = SD_JOB_SUCCESS;
                        break;
                      }
                    }
                  }

                  break;
                }
                default:
                {
                    // Unknown job type
                    currentJob->status = SD_JOB_FAILED;
                    currentJob->fresult = FR_INVALID_PARAMETER;
                    break;
                }
            }
        }
    }
  /* USER CODE END SD_Start */
}

/* USER CODE BEGIN Header_Playback_Start */
/**
* @brief Function implementing the Playback_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Playback_Start */
void Playback_Start(void const * argument)
{
  /* USER CODE BEGIN Playback_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Playback_Start */
}

/* USER CODE BEGIN Header_UART_Start */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Start */
void UART_Start(void const * argument)
{
  /* USER CODE BEGIN UART_Start */
  /* Infinite loop */
  char* msg;
  for(;;)
  {
    if (xQueueReceive(uartQueueHandle, &(msg), portMAX_DELAY)){
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        vPortFree(msg); 
    }
  }
  /* USER CODE END UART_Start */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
#ifdef USE_FULL_ASSERT
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
