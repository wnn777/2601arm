/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "gpio.h"
#include <math.h> 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_init.h"
#include "lcd.h"
#include "pic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CENTER_X 140      
#define CENTER_Y 120     
#define RADIUS 100         
#define SECOND_HAND_LEN 60 // 
#define MINUTE_HAND_LEN 40  
#define HOUR_HAND_LEN 20    
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern int s;
extern int m;
extern int h;
extern int Flag2msok;
extern int Flag1sok;
extern int old_h;
extern int old_m;
extern int old_s;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DrawClockFace() {
    
    Draw_Circle(CENTER_X, CENTER_Y, RADIUS, 0xFFFF); 

    
    for (int i = 0; i < 12; i++) {
        float angle = i * 30 * 3.14159 / 180; 
        uint16_t x_start = CENTER_X + (RADIUS - 10) * sin(angle); 
        uint16_t y_start = CENTER_Y - (RADIUS - 10) * cos(angle); 
        uint16_t x_end = CENTER_X + RADIUS * sin(angle);         
        uint16_t y_end = CENTER_Y - RADIUS * cos(angle);
        LCD_DrawLine(x_start, y_start, x_end, y_end, 0xFFFF);
		}

    for (int i = 0; i < 60; i++) {
        float angle = i * 6 * 3.14159 / 180; 
        uint16_t x_start1 = CENTER_X + (RADIUS - 5) * sin(angle); 
        uint16_t y_start1 = CENTER_Y - (RADIUS - 5) * cos(angle); 
        uint16_t x_end1 = CENTER_X + RADIUS * sin(angle);         
        uint16_t y_end1= CENTER_Y - RADIUS * cos(angle);
        LCD_DrawLine(x_start1, y_start1, x_end1, y_end1, 0xFFFF);			
    }
}


void DrawHands(uint8_t old_h, uint8_t old_m, uint8_t old_s, uint8_t new_h, uint8_t new_m, uint8_t new_s) {

    float second_angle_old = old_s * 6 * 3.14159 / 180; 
    float minute_angle_old = (old_m + old_s / 60.0) * 6 * 3.14159 / 180; 
    float hour_angle_old = (old_h % 12 + old_m / 60.0) * 30 * 3.14159 / 180; 


    float second_angle_new = new_s * 6 * 3.14159 / 180;
    float minute_angle_new = (new_m + new_s / 60.0) * 6 * 3.14159 / 180;
    float hour_angle_new = (new_h % 12 + new_m / 60.0) * 30 * 3.14159 / 180;


    LCD_DrawLine(CENTER_X, CENTER_Y, 
                 CENTER_X + SECOND_HAND_LEN * sin(second_angle_old),
                 CENTER_Y - SECOND_HAND_LEN * cos(second_angle_old),BLUE );
    LCD_DrawLine(CENTER_X, CENTER_Y, 
                 CENTER_X + MINUTE_HAND_LEN * sin(minute_angle_old),
                 CENTER_Y - MINUTE_HAND_LEN * cos(minute_angle_old),BLUE );
    LCD_DrawLine(CENTER_X, CENTER_Y, 
                 CENTER_X + HOUR_HAND_LEN * sin(hour_angle_old),
                 CENTER_Y - HOUR_HAND_LEN * cos(hour_angle_old),BLUE );

    LCD_DrawLine(CENTER_X, CENTER_Y, 
                 CENTER_X + SECOND_HAND_LEN * sin(second_angle_new),
                 CENTER_Y - SECOND_HAND_LEN * cos(second_angle_new),WHITE); 
    LCD_DrawLine(CENTER_X, CENTER_Y, 
                 CENTER_X + MINUTE_HAND_LEN * sin(minute_angle_new),
                 CENTER_Y - MINUTE_HAND_LEN * cos(minute_angle_new), WHITE); 
    LCD_DrawLine(CENTER_X, CENTER_Y, 
                 CENTER_X + HOUR_HAND_LEN * sin(hour_angle_new),
                 CENTER_Y - HOUR_HAND_LEN * cos(hour_angle_new), WHITE);     
}

void Drawnum(void){
    for (int i = 1; i <= 12; i++) {
        float angle = i * 30 * 3.14159 / 180; 
        uint16_t x_start = CENTER_X + (RADIUS+10) * sin(angle); 
        uint16_t y_start = CENTER_Y - (RADIUS+10) * cos(angle); 
        LCD_ShowIntNum(x_start,y_start,i,i/10+1,WHITE,BLUE,12);
		}
}

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();//LCD初始化
  LCD_Init();//LCD初始化
  LCD_Fill(0,0,LCD_W,LCD_H,BLUE);
	DrawClockFace();
	Drawnum();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//LCD_ShowPicture(50,50,150,79,gImage_1);
		//LCD_ShowPicture(50,129,150,79,gImage_1);
		if(Flag1sok==1){
			DrawHands(old_h,old_m,old_s,h,m,s);
			Flag1sok=0;

		}

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
