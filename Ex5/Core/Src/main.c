/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG(GPIO_TypeDef**GPIO_Port,uint16_t*GPIO_Pin,int num){
	if(num==0){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],0);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],0);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],0);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],0);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],0);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],1);
	}
	if(num==1){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],1);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],0);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],1);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],1);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],1);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],1);
	}
	if(num==2){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],0);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],0);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],1);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],0);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],0);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],1);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],0);
	}
	if(num==3){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],0);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],0);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],0);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],1);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],1);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],0);
	}
	if(num==4){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],1);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],0);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],1);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],1);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],0);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],0);
	}
	if(num==5){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],0);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],1);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],0);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],1);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],0);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],0);
	}
	if(num==6){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],0);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],1);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],0);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],0);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],0);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],0);
	}
	if(num==7){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],0);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],0);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],1);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],1);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],1);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],1);
	}
	if(num==8){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],0);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],0);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],0);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],0);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],0);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],0);
	}
	if(num==9){
		HAL_GPIO_WritePin(GPIO_Port[0],GPIO_Pin[0],0);
		HAL_GPIO_WritePin(GPIO_Port[1],GPIO_Pin[1],0);
		HAL_GPIO_WritePin(GPIO_Port[2],GPIO_Pin[2],0);
		HAL_GPIO_WritePin(GPIO_Port[3],GPIO_Pin[3],0);
		HAL_GPIO_WritePin(GPIO_Port[4],GPIO_Pin[4],1);
		HAL_GPIO_WritePin(GPIO_Port[5],GPIO_Pin[5],0);
		HAL_GPIO_WritePin(GPIO_Port[6],GPIO_Pin[6],0);
	}
}

void display_way(GPIO_TypeDef**GPIO_Port_Led, uint16_t*GPIO_Pin_Led,
		GPIO_TypeDef**GPIO_Port_LED7SEG, uint16_t*GPIO_Pin_LED7SEG,
		int*cnt_led_max,int*cnt_led, int*state, int max_state, int active, int inactive ){
	if(*state==0){
		HAL_GPIO_WritePin(GPIO_Port_Led[max_state], GPIO_Pin_Led[max_state], inactive);
		HAL_GPIO_WritePin(GPIO_Port_Led[*state], GPIO_Pin_Led[*state], active);
	}else{
		HAL_GPIO_WritePin(GPIO_Port_Led[(*state)-1], GPIO_Pin_Led[(*state)-1], inactive);
		HAL_GPIO_WritePin(GPIO_Port_Led[*state], GPIO_Pin_Led[*state], active);
	}
	display7SEG(GPIO_Port_LED7SEG, GPIO_Pin_LED7SEG, cnt_led[*state]);
	cnt_led[*state]--;
	if(cnt_led[*state]<=0){
		cnt_led[*state]=cnt_led_max[*state];
		if((*state)==max_state){
			*state=0;
		}else{
			(*state)++;
		}
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
  /* USER CODE BEGIN 2 */
  	int cnt_red_max=5;  //state 0
	int cnt_yellow_max=2;  //state 1
	int cnt_green_max=3; //state 2
	int cnt_led_max[]={cnt_red_max,cnt_yellow_max,cnt_green_max};

	/* WAY1 PORT PACKAGE */
	int cnt_red1=cnt_red_max;
	int cnt_yellow1=cnt_yellow_max;
	int cnt_green1=cnt_green_max;
	int cnt_led1[]={cnt_red1, cnt_yellow1, cnt_green1};
	int cur_state1=0;

	GPIO_TypeDef*GPIO_Port_Led_Way1[]={LED_RED1_GPIO_Port, LED_YELLOW1_GPIO_Port,
			LED_GREEN1_GPIO_Port};
	uint16_t GPIO_Pin_Led_Way1[]={LED_RED1_Pin, LED_YELLOW1_Pin, LED_GREEN1_Pin};

	GPIO_TypeDef*GPIO_Port_LED7SEG1[]={a1_GPIO_Port, b1_GPIO_Port, c1_GPIO_Port, d1_GPIO_Port,
			  e1_GPIO_Port, f1_GPIO_Port, g1_GPIO_Port};
	uint16_t GPIO_Pin_LED7SEG1[]={a1_Pin, b1_Pin, c1_Pin, d1_Pin, e1_Pin, f1_Pin, g1_Pin};
	/* END WAY1 PROT PACKAGE */

	/* WAY2 PORT PACKAGE */
	int cnt_red2=cnt_red_max;
	int cnt_yellow2=cnt_yellow_max;
	int cnt_green2=cnt_green_max;
	int cnt_led2[]={cnt_red2, cnt_yellow2, cnt_green2};
	int cur_state2=1;

	GPIO_TypeDef*GPIO_Port_Led_Way2[]={LED_RED2_GPIO_Port, LED_YELLOW2_GPIO_Port,
				LED_GREEN2_GPIO_Port};
	uint16_t GPIO_Pin_Led_Way2[]={LED_RED2_Pin, LED_YELLOW2_Pin, LED_GREEN2_Pin};

	GPIO_TypeDef*GPIO_Port_LED7SEG2[]={a2_GPIO_Port,b2_GPIO_Port,c2_GPIO_Port,d2_GPIO_Port,
			  e2_GPIO_Port, f2_GPIO_Port,g2_GPIO_Port};
	uint16_t GPIO_Pin_LED7SEG2[]={a2_Pin, b2_Pin, c2_Pin, d2_Pin, e2_Pin, f2_Pin, g2_Pin};
	/* END WAY2 PROT PACKAGE */

	for(int i=0;i<3;i++){
		HAL_GPIO_WritePin(GPIO_Port_Led_Way1[i], GPIO_Pin_Led_Way1[i], 0);
		HAL_GPIO_WritePin(GPIO_Port_Led_Way2[i], GPIO_Pin_Led_Way2[i], 0);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* WAY1 START */
	  display_way(GPIO_Port_Led_Way1, GPIO_Pin_Led_Way1,
			  GPIO_Port_LED7SEG1, GPIO_Pin_LED7SEG1,
			  cnt_led_max, cnt_led1,&cur_state1 ,2,1,0);
	  display_way(GPIO_Port_Led_Way2, GPIO_Pin_Led_Way2,
	  			  GPIO_Port_LED7SEG2, GPIO_Pin_LED7SEG2,
				  cnt_led_max, cnt_led2,&cur_state2 ,2,1,0);
	  /* WAY1 END */

	  /* WAY2 START */

	  /* WAY2 END */
	  HAL_Delay(1000);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED2_Pin|LED_YELLOW2_Pin|LED_GREEN2_Pin|LED_RED1_Pin
                          |LED_YELLOW1_Pin|LED_GREEN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a1_Pin|b1_Pin|c1_Pin|d2_Pin
                          |e2_Pin|f2_Pin|g2_Pin|d1_Pin
                          |e1_Pin|f1_Pin|g1_Pin|a2_Pin
                          |b2_Pin|c2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED2_Pin LED_YELLOW2_Pin LED_GREEN2_Pin */
  GPIO_InitStruct.Pin = LED_RED2_Pin|LED_YELLOW2_Pin|LED_GREEN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED1_Pin LED_YELLOW1_Pin LED_GREEN1_Pin */
  GPIO_InitStruct.Pin = LED_RED1_Pin|LED_YELLOW1_Pin|LED_GREEN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : a1_Pin b1_Pin c1_Pin d2_Pin
                           e2_Pin f2_Pin g2_Pin d1_Pin
                           e1_Pin f1_Pin g1_Pin a2_Pin
                           b2_Pin c2_Pin */
  GPIO_InitStruct.Pin = a1_Pin|b1_Pin|c1_Pin|d2_Pin
                          |e2_Pin|f2_Pin|g2_Pin|d1_Pin
                          |e1_Pin|f1_Pin|g1_Pin|a2_Pin
                          |b2_Pin|c2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
