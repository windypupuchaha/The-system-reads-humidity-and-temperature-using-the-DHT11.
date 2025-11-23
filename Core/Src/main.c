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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS_Pin  GPIO_PIN_1
#define E_Pin   GPIO_PIN_2
#define D4_Pin  GPIO_PIN_4
#define D5_Pin  GPIO_PIN_5
#define D6_Pin  GPIO_PIN_6
#define D7_Pin  GPIO_PIN_7
#define LCD_Port GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN PV */

//DHT11_DataTypedef dht;
//char str[20] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--------------------LCD----------------------LCD--------------------LCD--------------------
void LCD_Enable(void)
{
   HAL_GPIO_WritePin(LCD_Port, E_Pin, GPIO_PIN_SET);
   HAL_Delay(1);
   HAL_GPIO_WritePin(LCD_Port, E_Pin, GPIO_PIN_RESET);
   HAL_Delay(1);
}
void LCD_Send4Bits(uint8_t data)
{
   HAL_GPIO_WritePin(LCD_Port, D4_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
   HAL_GPIO_WritePin(LCD_Port, D5_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
   HAL_GPIO_WritePin(LCD_Port, D6_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
   HAL_GPIO_WritePin(LCD_Port, D7_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
   LCD_Enable();
}
void LCD_SendCmd(uint8_t cmd)
{
   HAL_GPIO_WritePin(LCD_Port, RS_Pin, GPIO_PIN_RESET);
   LCD_Send4Bits(cmd >> 4);
   LCD_Send4Bits(cmd & 0x0F);
}
void LCD_MoveCursor(uint8_t row, uint8_t col)
{
	uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
	LCD_SendCmd(address);
}
void LCD_SendChar(uint8_t data) {
   HAL_GPIO_WritePin(LCD_Port, RS_Pin, GPIO_PIN_SET); // Đặt chân RS lên 1 để gửi dữ liệu
   LCD_Send4Bits(data >> 4);                          // Gửi 4 bit cao
   LCD_Send4Bits(data & 0x0F);                        // Gửi 4 bit thấp
}
void LCD_Init(void) {
   HAL_Delay(20);      // đợi ổn định sau khi cấp nguồn
   // Gửi 0x03 (nibble) 3 lần để đảm bảo LCD về chế độ 8-bit (theo datasheet)
   HAL_GPIO_WritePin(LCD_Port, RS_Pin, GPIO_PIN_RESET);
   LCD_Send4Bits(0x03);
   HAL_Delay(5);
   LCD_Send4Bits(0x03);
   HAL_Delay(5);
   LCD_Send4Bits(0x03);
   HAL_Delay(1);
   // Gửi 0x02 để chuyển sang chế độ 4-bit
   LCD_Send4Bits(0x02);
   HAL_Delay(1);
   // Bây giờ LCD ở chế độ 4-bit, gửi các lệnh chuẩn
   LCD_SendCmd(0x28);  // Function set: 4-bit, 2 line, 5x8 dots
   HAL_Delay(1);
   LCD_SendCmd(0x0C);  // Display ON, cursor off, blink off
   HAL_Delay(1);
   LCD_SendCmd(0x01);  // Clear display
   HAL_Delay(2);       // Clear cần thời gian lâu hơn
   // (tùy chọn) Entry mode
   LCD_SendCmd(0x06);  // Entry mode: cursor moves right
   HAL_Delay(1);
}
void LCD_Puts(char* str)
{
   while (*str) {                              // Trong khi chuỗi chưa kết thúc (ký tự '\0')
       LCD_SendChar((uint8_t)(*str));          // Gửi ký tự hiện tại lên LCD
       str++;                                  // Di chuyển đến ký tự tiếp theo
   }
}

void lcd_clear (void)
{
	LCD_SendCmd(0x01);
	HAL_Delay(10);
}

void Display_Temp (float Temp)
{
	char str[20] = {0};
	LCD_MoveCursor(0, 0);

	sprintf (str, "TEMP: %2.1f ", Temp);
	LCD_Puts(str);
	LCD_Puts("C");
}

void Display_Rh (float Rh)
{
	char str[20] = {0};
	LCD_MoveCursor(1, 0);

	sprintf (str, "RH  : %2.1f ", Rh);
	LCD_Puts(str);
	LCD_Puts("%");
}


//--------------------DHT11----------------------DHT11--------------------DHT11--------------------
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_0

void TIM2_us_Init(void){
    // trong CubeMX set Prescaler = (SystemCoreClock/1000000)-1, CounterPeriod = 0xFFFF
    HAL_TIM_Base_Start(&htim2);
}

//void DELAY_us (uint16_t time_us)
//{
//	__HAL_TIM_SET_COUNTER(&htim2, 0);
//	HAL_TIM_Base_Start (&htim2);
//	htim2.Instance->CNT = 0;
//	while (htim2.Instance->CNT < time_us)
//	{
//		HAL_TIM_Base_Stop (&htim2);
//	}
//}

void DELAY_us(uint16_t us){
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while(__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start (void)
{

	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output

	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	HAL_Delay(18);   // wait for 18ms

    HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
    DELAY_us(20);   // wait for 20us
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	DELAY_us (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		DELAY_us (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	for (j=0;j<8;j++)
	{
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		DELAY_us (40);   // wait for 40 us
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		}
		else{
			i|= (1<<(7-j));  // if the pin is high, write 1
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		}
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}

void GPIO_Output_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void OutputNibble(uint8_t data)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
	uint8_t SUM, RH, TEMP;

	float Temperature = 0;
	float Humidity = 0;
	uint8_t Presence;
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
  MX_TIM2_Init();
  HAL_TIM_Base_Start(&htim2);
  LCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  LCD_MoveCursor(0, 1);
  LCD_Puts("Nhom 12 - L14");
  HAL_Delay(2000);
  lcd_clear ();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */
	  /* USER CODE END WHILE */

	        /* USER CODE BEGIN 3 */
//	  	  Temperature+=1;
	  lcd_clear ();
	    	Display_Temp(Temperature);
	    	Display_Rh(Humidity);

	        HAL_Delay(1000);


	        DHT11_Start();
	        Presence = DHT11_Check_Response();

	        // 4. KIỂM TRA PHẢN HỒI
	        if (1)
	        {

	            Rh_byte1 = DHT11_Read();
	            Rh_byte2 = DHT11_Read();
	            Temp_byte1 = DHT11_Read();
	            Temp_byte2 = DHT11_Read();
//	            SUM = DHT11_Read();
	            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
//		    	Display_Temp(Temperature);
//		    	Display_Rh(Humidity);
//	            OutputNibble(Rh_byte1);   // hiển thị RH byte 1
//	            HAL_Delay(1000);
//
//	            OutputNibble(Rh_byte2);   // hiển thị RH byte 2
//	            HAL_Delay(1000);

//	            OutputNibble(Rh_byte1); // hiển thị Temp byte 1
	            //HAL_Delay(1000);

//	            OutputNibble(Temp_byte2); // hiển thị Temp byte 2
//	            HAL_Delay(1000);
//
//	            OutputNibble(SUM);        // hiển thị Checksum
//	            HAL_Delay(1000);
	            // 5. KIỂM TRA CHECKSUM
//	            if (SUM == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2))
	            if (1)
	            {
	                RH = Rh_byte1;
	                TEMP = Temp_byte1;

	                Temperature = (float)TEMP;
	                Humidity = (float)RH;
	                HAL_Delay(1000);
	            }

	            else
	            {
	                // Checksum lỗi → giữ giá trị cũ
	                // Có thể hiển thị "ERR"
	                LCD_MoveCursor(1, 10);
	                LCD_Puts("ERR");
	            }
	        }
	        else
	        {
	            // Không có phản hồi → giữ giá trị cũ
	            LCD_MoveCursor(1, 10);
	            LCD_Puts("NO ");
	        }
	      }


    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Outpu++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++0
   * t Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA4 PA5
                           PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
