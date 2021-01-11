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
#include "string.h"
#include <IR_Remote.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STR_LEN 300
#define COMMAND_COUNT 11
#define REMOTE_NAME_LEN 20
#define COMMAND_NAME_LEN 10
#define ESP_LEN 3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t recBuffer[1];
uint8_t customBuffer[STR_LEN];
uint8_t nextionMessageBuffer[STR_LEN];
uint8_t espRecBuffer[1];
uint8_t espBuffer[ESP_LEN];
uint8_t lastAvailableIndex = 0;
uint8_t lastAvailableEspIndex = 0;
int isBufferReady = 0;
int isEspBufferReady = 0;
int hasValidStart = 0;
int espRequest = 0;

char currentRemoteName[20];
remote_data currentRemote[COMMAND_COUNT];
full_remote_data currentRemoteFull;
part_remote_data remoteList;
HAL_StatusTypeDef status;

char currentButtonName[COMMAND_NAME_LEN];
int remoteOperation = 0;
int buttonIndex = -1;
int recName = 0;
int recSelect = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void openRemote(int index);

void getRemoteListAPI();
full_remote_data getRemoteAPI(char *remoteName);

void convertCommandToLocal(int buttonIndex);
void sendNextion(char *data);
void clearRemote();
void putRemote(int isReady);
void getRemote();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_IT(&huart1, recBuffer, 1);
	HAL_UART_Receive_IT(&huart6, espRecBuffer, 1);

	clearRemote();
	memset(nextionMessageBuffer, 0, sizeof(nextionMessageBuffer));
	HAL_Delay(500); // Wait for nextion EEPROM
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// If we have valid button use it.
		if (remoteOperation > 0 && buttonIndex >= 0) {
			int opResult = 0;
			if (remoteOperation == 1) {
				opResult = sendButton(buttonIndex);
			} else if (remoteOperation == 2) {
				opResult = recButton(buttonIndex);
			}
			remoteOperation = 0;
		}

		if (recSelect && buttonIndex >= 0) {
			// TODO: Use pagination * list size + buttonIndex if using pagination...
			if (remoteList.remoteCount > buttonIndex) {
				openRemote(buttonIndex);
			}
			recSelect = 0;
		}

		if (!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
			// DEBUG Command
			//clearNextion();
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			HAL_Delay(500);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			getRemoteListAPI();
			if (status == HAL_OK) {
				getRemoteAPI(remoteList.remotes[0]);
			}
		}
		HAL_Delay(500);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 99;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 9;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_9B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_ODD;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : IR_In_Pin */
	GPIO_InitStruct.Pin = IR_In_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IR_In_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void toggleSettingMode() {
	sendNextion("click b_setting,0");
	sendNextion("click b_setting,1");
}

int isCommandValid(full_command_data command) {
	return command.decode_type >= 0 && command.decode_type < 17;
}

void openRemote(int index) {
	sendNextion("page page1");
	toggleSettingMode();
	sendNextion("t0.txt=\"Please Wait. Loading Remote...\"");
	getRemoteAPI(remoteList.remotes[index]);

	if (currentRemoteFull.name[0] == remoteList.remotes[index][0]) {
		toggleSettingMode();
		char buf[100];
		sprintf(buf, "t1.txt=\"%s\"", currentRemoteFull.name);
		sendNextion(buf);
		for (int i = 0; i < COMMAND_COUNT - 1; i++) {
			if (!isCommandValid(currentRemoteFull.commands[i])) {
				continue;
			}
			sprintf(buf, "b%d.txt=\"%s\"", i,
					currentRemoteFull.commands[i].name);
			sendNextion(buf);
		}
	} else {
		sendNextion("t0.txt=\"ERROR Loading Remote. Try Again.\"");
	}
}

// CALL API Functions in main only.
void getRemoteListAPI() {
	char espMessage[100];
	sprintf(espMessage, "grl");
	HAL_UART_AbortReceive_IT(&huart6);
	HAL_UART_Transmit(&huart6, espMessage, strlen(espMessage), 2000);
	status = HAL_UART_Receive(&huart6, &espBuffer, ESP_LEN, 20000); // Maybe dont block here.
	if (status == HAL_OK) {
		memset(&remoteList, 0, sizeof(remoteList));
		memcpy(&remoteList, espBuffer, sizeof(remoteList));
	}
	HAL_UART_Receive_IT(&huart1, recBuffer, 1);
}

void convertRemoteToLocal() {
	for (int i = 0; i < COMMAND_COUNT; i++) {
		convertCommandToLocal(i);
	}
	memcpy(currentRemoteName, currentRemoteFull.name, REMOTE_NAME_LEN);
	// TODO: PUTREMOTE HERE....
}

full_remote_data getRemoteAPI(char *remoteName) {
	if (strcmp(remoteName, currentRemoteFull.name) == 0) {
		return currentRemoteFull;
	}

	char espMessage[100];
	sprintf(espMessage, "grs:%s", remoteName);
	HAL_UART_AbortReceive_IT(&huart6);
	HAL_UART_Transmit(&huart6, espMessage, strlen(espMessage), 2000);
	status = HAL_UART_Receive(&huart6, &espBuffer, ESP_LEN, 10000); // Maybe dont block here.
	memcpy(&currentRemoteFull, espBuffer, sizeof(currentRemoteFull));

	HAL_UART_Receive_IT(&huart1, recBuffer, 1);
	return currentRemoteFull;
}

void clearNextion() {
	clearRemote();
	currentRemote[0].decode_type = 13;
	sprintf(currentRemoteName, "HOOOLA");
	putRemote(0);
}

void clearRemote() {
	memset(currentRemote, 0, sizeof(currentRemote));
	for (int i = 0; i < COMMAND_COUNT; i++) {
		currentRemote[i].decode_type = UNUSED;
	}
}

void sendNextion(char *data) {
	char nextionBuf[100];
	sprintf(nextionBuf, "%s%c%c%c", data, 0xff, 0xff, 0xff);
	HAL_UART_Transmit(&huart1, nextionBuf, strlen(nextionBuf), 1000);
}

void sendRemoteToNextion() {
	char nextionBuf[1024];
	HAL_UART_Transmit(&huart1, currentRemoteName, REMOTE_NAME_LEN, 2000);
	memcpy(nextionBuf, currentRemote, sizeof(currentRemote));
	HAL_UART_Transmit(&huart1, nextionBuf, sizeof(currentRemote), 2000);
}

void putRemote(int isReady) {
	if (isReady) {
		sendRemoteToNextion();
	} else {
		char storeNextionBuf[30];
		sprintf(storeNextionBuf, "wept 0,%d",
				sizeof(currentRemote) + REMOTE_NAME_LEN);
		sendNextion(storeNextionBuf);
	}
}

void getRemote() {
	char recNextionBuf[1000];
	sprintf(recNextionBuf, "rept 0,%d",
			sizeof(currentRemote) + REMOTE_NAME_LEN);

	// Disable uart receive it temporarily.
	HAL_UART_AbortReceive_IT(&huart1);
	sendNextion(recNextionBuf);

	// Receive sync
	HAL_UART_Receive(&huart1, &recNextionBuf,
			sizeof(currentRemote) + REMOTE_NAME_LEN, 2000);
	memcpy(currentRemoteName, recNextionBuf, REMOTE_NAME_LEN);
	memcpy(currentRemote, &recNextionBuf[20], sizeof(currentRemote));

	// Continue to receive
	HAL_UART_Receive_IT(&huart1, recBuffer, 1);
	sprintf(recNextionBuf, "t1.txt=\"%s\"", currentRemoteName);
	sendNextion(recNextionBuf);
	for (int i = 0; i < COMMAND_COUNT - 1; i++) {
		sprintf(recNextionBuf, "b%d.txt=\"%s\"", i, currentRemote[i].name);
		sendNextion(recNextionBuf);
	}
}

void sendButton(int buttonIndex) {
	if (currentRemote[buttonIndex].decode_type == UNUSED) {
		// Command not loaded.
		sendNextion("t0.txt=\"Command not loaded!!!\"");
		return;
	}

	remote_data currentCommand = currentRemote[buttonIndex];
	for (int i = 0; i < 2; i++) {
		// Send twice, just in case...
		send(NULL, currentCommand.rawLen, currentCommand.value,
				currentCommand.bitLen, currentCommand.decode_type);
		HAL_Delay(50);
	}

	sendNextion("t0.txt=\"Button sent!\"");
	return;
}

void convertCommandToLocal(int buttonIndex) {
	remote_data remote;
	full_command_data currentCommand = currentRemoteFull.commands[buttonIndex];
	remote.bitLen = currentCommand.bitLen;
	remote.decode_type = currentCommand.decode_type;
	remote.rawLen = currentCommand.rawLen;
	memcpy(remote.name, currentCommand.name, sizeof(currentCommand.name))
}

void recButton(int buttonIndex) {
	my_enableIRIn();
	int i = 10; // Wait 10 sec to receive command
	while (i) {
		char nextionBuffer[100];
		sprintf(nextionBuffer, "t0.txt=\"Waiting for %d seconds...\"", i);
		sendNextion(nextionBuffer);
		if (my_decode(&results)) {
			my_disable();

			char nextionBuffer[100];
			sprintf(nextionBuffer, "t0.txt=\"PROTOCOL: %s\"",
					getProtocolString(results.decode_type));
			sendNextion(nextionBuffer);

			if (results.decode_type >= 0) {
				full_command_data command;
				command.decode_type = results.decode_type;
				command.rawLen = results.rawlen;
				command.bitLen = results.bits;
				command.value = results.value;
				command.pos = buttonIndex;
				for (int i = 0; i < command.rawLen; i++) {
					command.raw[i] = results.rawbuf[i];
				}
				memset(command.name, 0, sizeof(command.name));
				memcpy(command.name, currentButtonName,
						sizeof(currentButtonName));

				currentRemoteFull.commands[buttonIndex] = command;
				convertCommandToLocal(buttonIndex);
			}

			if (buttonIndex == 10) {
				memset(currentRemoteName, 0, sizeof(currentRemoteName));
				memcpy(currentRemoteName, currentButtonName,
						strlen(currentButtonName));
			}

			convertCommandToLocal(buttonIndex);
			putRemote(0);
			HAL_Delay(1000);
			if (buttonIndex < 10) {
				sprintf(nextionBuffer, "t0.txt=\"Button %d saved.\"",
						buttonIndex);
			} else {
				sprintf(nextionBuffer, "t0.txt=\"Master button saved.\"");
			}
			sendNextion(nextionBuffer);
			break;
		}
		HAL_Delay(1000);
		i--;
	}

	// Toggle setting back. (Touch press and release)
	toggleSettingMode();

	// No data received.
	return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (isBufferReady) {
			// Cleanup old buffer, new data incoming !!!
			isBufferReady = 0;
			lastAvailableIndex = 0;
			memset(customBuffer, 0, STR_LEN);
		}

		customBuffer[lastAvailableIndex] = recBuffer[0];
		uint8_t i = lastAvailableIndex;
		lastAvailableIndex++;

		// Check if its last. Nextion uses 3 '0xff' as last (set custom buffer as available)
		if (i > 1 && customBuffer[i] == 0xff && customBuffer[i - 1] == 0xff
				&& customBuffer[i - 2] == 0xff) {
			isBufferReady = 1;
			customBuffer[i] = 0;
			customBuffer[i - 1] = 0;
			customBuffer[i - 2] = 0;
		}

		// UART Buffer Ready, Do sth.
		if (isBufferReady) {
			remoteOperation = 0;
			if (recName) {
				memset(currentButtonName, 0, sizeof(currentButtonName));
				memcpy(currentButtonName, customBuffer, strlen(customBuffer));
				remoteOperation = 2;
				recName = 0;
			} else if (strncmp("$send", customBuffer, strlen("$send")) == 0) {
				remoteOperation = 1;
			} else if (strncmp("$rec", customBuffer, strlen("$rec")) == 0) {
				recName = 1;
			} else if (strncmp("$list", customBuffer, strlen("$list")) == 0) {
				if (remoteList.remoteCount > 0) {
					for (int i = 0; i < remoteList.remoteCount; ++i) {
						if (i > 7) {
							break;
						}
						char buf[100];
						sprintf(buf, "b%d.txt=\"%s\"", i + 2,
								remoteList.remotes[i]);
						sendNextion(buf);
					}
				} else {
					sendNextion("b2.txt=\"No remote.\"");
					sendNextion("b3.txt=\"Or still loading..\"");
				}
			} else if (strncmp("$rem", customBuffer, strlen("$rem")) == 0) {
				recSelect = 1;
			} else if (customBuffer[0] == 0xfe) {
				// Nextion ready to receive data.
				putRemote(1);
			} else if (customBuffer[0] == 0xfd) {
				if (strlen(nextionMessageBuffer) > 0) {
					sendNextion(nextionMessageBuffer);
					memset(nextionMessageBuffer, 0,
							sizeof(nextionMessageBuffer));
				}
				getRemote();
			} else if (strncmp("$last", customBuffer, strlen("$last")) == 0) {
				getRemote();
			}

			if (remoteOperation == 1 || recName || recSelect) {
				// Determine which button to use for remote operation.
				buttonIndex = -1;
				uint8_t *bIndex = strchr(customBuffer, 'b');
				if (strstr(customBuffer, "master") != NULL) {
					buttonIndex = 10;
				} else {
					int i = (int) (bIndex - customBuffer) + 1;
					buttonIndex = atoi(&customBuffer[i]);
				}
			}
		}

		// Always wait for another byte
		HAL_UART_Receive_IT(&huart1, recBuffer, 1);
	} else if (huart->Instance == USART6) {
		// Always receive errors.
		HAL_UART_Receive_IT(&huart6, espRecBuffer, 1);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
