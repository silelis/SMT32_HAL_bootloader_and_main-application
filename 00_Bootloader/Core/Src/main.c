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
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <FLASH_SECTOR.h_bak>		//For Sector flash
//#include "FLASH_PAGE.h"			//For page flash

#define APPLICATION_ADDRESS			0x08010000						// Main application start address
#define APPLICATION_END_ADDRESS		0x081FFFFF-1					// Main application end address
#define USBMountDelayCounter		254		/*65534*/
#define USB_DEV_power_on(USB_power_on_port, USB_power_on_pin)		HAL_GPIO_WritePin(USB_power_on_port, USB_power_on_pin, GPIO_PIN_SET)
#define USB_DEV_power_off(USB_power_on_port, USB_power_on_pin)		HAL_GPIO_WritePin(USB_power_on_port, USB_power_on_pin, GPIO_PIN_RESET)

#include "SileliS_printf.h"
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static uint32_t FlashGetSector(uint32_t Address);
static uint32_t FlashEraseData(uint32_t StartSectorAddress, uint32_t EndSectorAddress);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



typedef void (*pFunction)(void);
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  printf_init(&huart3);
  printf("Bootloader v0.90\r\n");
  HAL_Delay(1500);	//Delay for power on button long press detection //TODO: powinno byc 3 sekundy
  //TODO: załączenie pinu podtrzymującego EN zasilania

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* FIRMWARE UPDATE CODE */
  if (HAL_GPIO_ReadPin(User_button_GPIO_Port, User_button_Pin))		//Enter flashing: Should be user own condition. //TODO: remember to change hardware configuration for final design
  {
	  USB_DEV_power_on(GPIOG, GPIO_PIN_6);
	  HAL_Delay(150);

		extern char USBHPath[4]; /* USBH logical drive path */
		extern FATFS USBHFatFS; /* File system object for USBH logical drive */
		extern ApplicationTypeDef Appli_state;

		uint8_t USBConnectionCounter = 0;

		do {
			MX_USB_HOST_Process();
			switch (Appli_state){
			case   APPLICATION_READY:

				/* Is USB mounted */
				if (f_mount(&USBHFatFS, (const TCHAR*) USBHPath, 0)== FR_OK)
				{

					/* Check if firmware file is avaliable*/
					FIL fp;
					if (f_open(&fp, "firmware.bin", FA_READ|FA_OPEN_EXISTING) == FR_OK) {

						printf("Firmware update\r\n");
						uint32_t toFlash;
						UINT brCounetr=0;
						__IO uint32_t binFileAddress = 0;

						/*Unlock flash, erase sectors, write new firmware*/
						HAL_FLASH_Unlock();
						if (FlashEraseData(APPLICATION_ADDRESS, APPLICATION_END_ADDRESS)==0);
						{
							do
							{
								toFlash=0xffffffff;
								f_read(&fp, (uint32_t*) &toFlash, sizeof(uint32_t), &brCounetr);

								/*chech if file had been read if "yes" write firmware*/
								if (brCounetr!=0)
								{
									uint8_t* toFlashBigEndian;
									toFlashBigEndian = &toFlash;
									printf("Flash:0x%.8x bin:0x%.8x 0x%.2x%.2x%.2x%.2x\r\n", APPLICATION_ADDRESS+binFileAddress, binFileAddress, *(toFlashBigEndian), *(toFlashBigEndian+1), *(toFlashBigEndian+2), *(toFlashBigEndian+3));
									HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APPLICATION_ADDRESS+binFileAddress, *&toFlash);
									binFileAddress+=sizeof(toFlash);
								}

								/* if file had not been read*/
								else
								{
									/* Check if USB stick is plugged */
									HAL_Delay(250);
									MX_USB_HOST_Process();

									/* USB stick is pluged. Firmware file end reached. */
									if (Appli_state==APPLICATION_READY)	//jeśli koniec odczytu nie nastąpił w skutek usterki / wyjęcia nośnika USB
									{
										//TODO: SPRAWDZANIE POPRAWNOŚCI WGRANIA FLASHA

										/*close file, unmount partition*/
										f_close(&fp);
										f_mount(0, (const TCHAR*) USBHPath/*""*/, 0);
										USBConnectionCounter = USBMountDelayCounter;	//To went out of loop which check if USB stick is plugged
										printf("EOF\r\n");
									}

									/* USB stick not plugged */
									else
									{
										/*close file, unmount partition, notify user about error, erase partition, lock flash, emergency mcu restart*/
										f_close(&fp);
										f_mount(0, (const TCHAR*) USBHPath, 0); //Na sam koniec bootloader powinien odmontować napęd USB
										HAL_UART_Transmit(&huart3, "FATAL ERROR.No USB.", 19, HAL_MAX_DELAY);	//printf("FATAL ERROR.No USB.");	// UART_Transmit is better because NVIC wait for UART_Transmit to be executed
										FlashEraseData(APPLICATION_ADDRESS, APPLICATION_END_ADDRESS);
										HAL_FLASH_Unlock();

										//TODO: Jeśli nie będzie działać poprawnie to zrobić power off poprzez wyłaczenie pinu EN podtrzymującego zasilacz oraz power off zasilania USB
										//TODO: Jakiś komunikat na HMI
										NVIC_SystemReset();
									}
									break;
								}
							}while(1);
						}
						HAL_FLASH_Lock();
					}

					/* If USB had not been mounted */
					else
					{
						printf("No firmware on USB.\r\n Check firmware filename: \"firmware.bin\"\r\n");
						HAL_Delay(1500);
					}
					break;
				}
            /* delay loop which gives time to user to plug USB stick */
			default:
				USBConnectionCounter++;
				HAL_Delay(10);
				if (USBConnectionCounter == USBMountDelayCounter)
					printf("No USB.\r\n");
			}
		} while (USBConnectionCounter < USBMountDelayCounter);

		HAL_FLASH_Lock();
		f_mount(0, (const TCHAR*) USBHPath, 0);
		USB_DEV_power_off(GPIOG, GPIO_PIN_6);
		HAL_Delay(150);
	}
  /* FIRMWARE UPDATE CODE */


  /* BOOTING MAIN APPLICATION*/

  	/*Check if main application area is blank.
  	 * if YES	*/
	if ((*(__IO uint32_t *)APPLICATION_ADDRESS==0xFFFFFFFF)&&(*(__IO uint32_t *)(APPLICATION_ADDRESS+1)==0xFFFFFFFF))
	{

		HAL_UART_Transmit(&huart3, "Main application area is blank\r\n", 32, HAL_MAX_DELAY);	//printf("Main application area is blank"); // UART_Transmit is better because NVIC wait for UART_Transmit to be executed
		//TODO:  komunikat o czystym obszarze flashana HMI

		//TODO: TUTAJ B�?DZIE LEPIEJ WYLĄCZYĆ PODTRZYMANIE EN ZASILANIA
		//TODO: Jakiś komunikat na HMI
		NVIC_SystemReset();
	}

	/*If main application area is flashed */
	else
	{
	 	printf("Start MainApp at:0x%.8x\r\n", APPLICATION_ADDRESS);
		pFunction appEntry;
		uint32_t appStack;

		//TODO: Wyłączenie wszystkich niepotrzebnych peryferiow

		/* Get the application stack pointer (First entry in the application vector table) */
		appStack = (uint32_t) *((__IO uint32_t*)APPLICATION_ADDRESS);

		/* Get the application entry point (Second entry in the application vector table) */
		appEntry = (pFunction) *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);

		/* Reconfigure vector table offset register to match the application location */
		SCB->VTOR = APPLICATION_ADDRESS;

		/* Set the application stack pointer */
		__set_MSP(appStack);

		/* Start the application */
		appEntry();
	}
	/* BOOTLOADER END*/



  while (1);
 // {
    /* USER CODE END WHILE */
//    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
 // };
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : User_button_Pin */
  GPIO_InitStruct.Pin = User_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF6 PF7
                           PF8 PF9 PF10 PF11
                           PF12 PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG5 PG8 PG9
                           PG10 PG11 PG12 PG13
                           PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13
                           PD14 PD15 PD0 PD1
                           PD2 PD3 PD4 PD5
                           PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static uint32_t FlashGetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < 0x08003FFF) && (Address >= 0x08000000))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < 0x08007FFF) && (Address >= 0x08004000))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < 0x0800BFFF) && (Address >= 0x08008000))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < 0x0800FFFF) && (Address >= 0x0800C000))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < 0x0801FFFF) && (Address >= 0x08010000))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < 0x0803FFFF) && (Address >= 0x08020000))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < 0x0805FFFF) && (Address >= 0x08040000))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < 0x0807FFFF) && (Address >= 0x08060000))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < 0x0809FFFF) && (Address >= 0x08080000))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < 0x080BFFFF) && (Address >= 0x080A0000))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < 0x080DFFFF) && (Address >= 0x080C0000))
  {
    sector = FLASH_SECTOR_10;
  }
  else if((Address < 0x080FFFFF) && (Address >= 0x080E0000))
  {
    sector = FLASH_SECTOR_11;
  }
  else if((Address < 0x08103FFF) && (Address >= 0x08100000))
  {
    sector = FLASH_SECTOR_12;
  }
  else if((Address < 0x08107FFF) && (Address >= 0x08104000))
  {
    sector = FLASH_SECTOR_13;
  }
  else if((Address < 0x0810BFFF) && (Address >= 0x08108000))
  {
    sector = FLASH_SECTOR_14;
  }
  else if((Address < 0x0810FFFF) && (Address >= 0x0810C000))
  {
    sector = FLASH_SECTOR_15;
  }
  else if((Address < 0x0811FFFF) && (Address >= 0x08110000))
  {
    sector = FLASH_SECTOR_16;
  }
  else if((Address < 0x0813FFFF) && (Address >= 0x08120000))
  {
    sector = FLASH_SECTOR_17;
  }
  else if((Address < 0x0815FFFF) && (Address >= 0x08140000))
  {
    sector = FLASH_SECTOR_18;
  }
  else if((Address < 0x0817FFFF) && (Address >= 0x08160000))
  {
    sector = FLASH_SECTOR_19;
  }
  else if((Address < 0x0819FFFF) && (Address >= 0x08180000))
  {
    sector = FLASH_SECTOR_20;
  }
  else if((Address < 0x081BFFFF) && (Address >= 0x081A0000))
  {
    sector = FLASH_SECTOR_21;
  }
  else if((Address < 0x081DFFFF) && (Address >= 0x081C0000))
  {
    sector = FLASH_SECTOR_22;
  }
  else if ((Address < 0x081FFFFF) && (Address >= 0x081E0000))
  {
    sector = FLASH_SECTOR_23;
  }
  return sector;
}

static uint32_t FlashEraseData(uint32_t StartSectorAddress, uint32_t EndSectorAddress)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError=0;


	/* Get the number of sector to erase from 1st sector */
	  uint32_t StartSector = FlashGetSector(StartSectorAddress);
	  //uint32_t EndSectorAddress = StartSectorAddress + numberofwords*4;
	  uint32_t EndSector = FlashGetSector(EndSectorAddress);

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector        = StartSector;
	  EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;

	  printf("Erasing flash\r\n");
	  HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
	  printf("Flash erased\r\n");
	  return HAL_FLASH_GetError ();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
