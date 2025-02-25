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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bitmaps.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADDR 0x78
#define WIDTH 130
#define HEIGHT 64
static uint8_t Buffer[WIDTH * HEIGHT / 8];

void mikrosekunda (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

uint32_t vrijednost1 = 0;
uint32_t vrijednost2 = 0;
uint32_t razlika = 0;
uint8_t prvi = 0;
uint8_t udaljenost  = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if (prvi==0)
			{
				vrijednost1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				prvi = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (prvi==1){
				vrijednost2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				__HAL_TIM_SET_COUNTER(htim, 0);

				if (vrijednost2 > vrijednost1){
					razlika = vrijednost2-vrijednost1;
				}

				else if (vrijednost1 > vrijednost2){
					razlika = (0xffff - vrijednost1) + vrijednost2;
				}

				udaljenost = razlika * .034/2;
				prvi = 0;

				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}

}

void UZ_Read (void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	mikrosekunda(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  uint16_t switchState;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

HAL_Delay(100);
//provjera konekcije OLED-a s I2C
  	if (HAL_I2C_IsDeviceReady(&hi2c1,ADDR, 1, 20000) != HAL_OK) {
  			return 0;
  		}
  	//Inicijaliziranje OLED-a
  	Write(0x78,0x00,0xAE); // display OFF
  	Write(0x78,0x00,0x20); //Memory Addressing mode
  	Write(0x78,0x00,0x10); //Page Addressing mode
  	Write(0x78,0x00,0xB0); //Page start address
  	Write(0x78,0x00,0xC8); //COM output scan direction
  	Write(0x78,0x00,0x00); // low column address
  	Write(0x78,0x00,0x10); // high column address
  	Write(0x78,0x00,0x40); // start line address
  	Write(0x78,0x00,0x81); // contrast register
  	Write(0x78,0x00,0xFF);
  	Write(0x78,0x00,0xA1); // segment re-map 0 do 127
  	Write(0x78,0x00,0xA6); //normal display

  	Write(0x78,0x00,0xA8); //Height OxFF za 128 px, 0xA8 za sve ostalo
  	Write(0x78,0x00,0x3F); // multiplex ratio

  	Write(0x78,0x00,0xA4); //Output follows RAM
  	Write(0x78,0x00,0xD3); // display offset
  	Write(0x78,0x00,0x00); // not offset
  	Write(0x78,0x00,0xD5); // frequency
  	Write(0x78,0x00,0xF0); // divide ratio
  	Write(0x78,0x00,0xD9); // pre-charge period
  	Write(0x78,0x00,0x22);
  	Write(0x78,0x00,0xDA); // COM pin
  	Write(0x78,0x00,0x12);

  	Write(0x78,0x00,0xDB); // Voltage pin (vcomh)
  	Write(0x78,0x00,0x20);
  	Write(0x78,0x00,0x8D); // DC-DC upaljen
  	Write(0x78,0x00,0x14);
  	Write(0x78,0x00,0xAF); //upali panel
  	//deaktiviraj scroll
  	Write(0x78,0x00,0x2E);
  	//Clear ekran
  	memset(Buffer, (0xFF) ? 0x00 : 0xFF, sizeof(Buffer));
  	//Update ekran
  	uint8_t i;

  		for (i = 0; i < 8; i++) {
  			Write(0x78,0x00,0xB0+i);
  			Write(0x78,0x00,0x00);
  			Write(0x78,0x00,0x10);

  			WriteMulti(ADDR, 0x40, &Buffer[WIDTH * i], WIDTH);
  			}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  UZ_Read();
	  	  HAL_Delay(200);
	  	  switchState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

	  	           if(switchState == 1){
	  	        	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	  	        	   memset(Buffer, (0xFF) ? 0x00 : 0xFF, sizeof(Buffer));
	  	        	   WriteBitmap(0,0,Priblizite_se_vratima,128,64,1);


	  	        	   	for (i = 0; i < 8; i++){
	  	        	   		Write(0x78,0x00,0xB0+i);
	  	        	   		Write(0x78,0x00,0x00);
	  	        	   		Write(0x78,0x00,0x10);

	  	        	   		WriteMulti(ADDR, 0x40, &Buffer[WIDTH * i], WIDTH);
	  	        	   		}
	  	        	   	if(udaljenost < 10){
	  	        	  	  	           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	  	        	  	  	     memset(Buffer, (0xFF) ? 0x00 : 0xFF, sizeof(Buffer));
	  	        	  	  	     WriteBitmap(0,0,Pricekajte,128,64,1);

	  	        	  	  	     	 	 for (i = 0; i < 8; i++) {
	  	        	  	  	     	  	      	  Write(0x78,0x00,0xB0+i);
	  	        	  	  	     	  	      	  Write(0x78,0x00,0x00);
	  	        	  	  	     	  	      	  Write(0x78,0x00,0x10);
	  	        	  	  	     	  	      	  WriteMulti(ADDR, 0x40, &Buffer[WIDTH * i], WIDTH);
	  	        	  	  	     	 	 	 	  }

	  	        	  	  	           HAL_Delay(1000);
	  	      	  	        	   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,2000);
	  	      	  	        	   HAL_Delay(500);
	  	      	  	        	memset(Buffer, (0xFF) ? 0x00 : 0xFF, sizeof(Buffer));
	  	      	  	        	WriteBitmap(0,0,Prodite,128,64,1);

	  	      	  	        			for (i = 0; i < 8; i++) {
	  	      	  	        					Write(0x78,0x00,0xB0+i);
	  	      	  	        		  	   		Write(0x78,0x00,0x00);
	  	      	  	        		  	   		Write(0x78,0x00,0x10);
	  	      	  	        		  	   		WriteMulti(ADDR, 0x40, &Buffer[WIDTH * i], WIDTH);
	  	      	  	        		  	   		}


	  	      	  	        	HAL_Delay(5000);
	  	      	  	        	memset(Buffer, (0xFF) ? 0x00 : 0xFF, sizeof(Buffer));

	  	      	  	        	for (i = 0; i < 8; i++) {
	  	        	   	        	   		Write(0x78,0x00,0xB0+i);
	  	        	   	        	   		Write(0x78,0x00,0x00);
	  	        	   	        	   		Write(0x78,0x00,0x10);
	  	        	   	        	   		WriteMulti(ADDR, 0x40, &Buffer[WIDTH * i], WIDTH);
	  	        	   	        	   		}
	  	        	   	}else{
	  	        	   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1000);

	  	           }
	  	           }else{
	  	           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	  	           __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1000);
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

/* USER CODE BEGIN 4 */
void Write(uint8_t address, uint8_t reg, uint8_t data) {
 	uint8_t a[2];
 	a[0] = reg;
 	a[1] = data;
 	HAL_I2C_Master_Transmit(&hi2c1, address, a, 2, 10);
 }

void WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t a[256];
	a[0] = reg;
	uint8_t i;
	for(i = 0; i < count; i++)
			a[i+1] = data[i];
			HAL_I2C_Master_Transmit(&hi2c1, address, a, count+1, 10);
	}

void WritePixel(uint16_t x, uint16_t y,uint16_t color) {
	if (color == 1) {
			Buffer[x + (y / 8) * WIDTH] |= 1 << (y % 8);
		} else {
			Buffer[x + (y / 8) * WIDTH] &= ~(1 << (y % 8));
		}
	}

void WriteBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color){

    int16_t byteWidth = (w+7)/8;
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++){
        for(int16_t i=0; i<w; i++){
            if(i & 7){
               byte <<= 1;
            }else{
            	byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }if(byte & 0x80)
            	WritePixel(x+i, y, color);
        }
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
