/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "oledd.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define wait 0    //等待状态
#define start 1   //开始状态
#define data_rx 2 //数据接收状态
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */            
uint8_t state=0;
uint8_t signal_repeat=0;
uint8_t signal_data_rx_over=0;
uint32_t count=0;
uint16_t pdata=0;
uint8_t ir_receive[4];
char expression[20] = {0};
uint8_t expr_len = 0;
int result = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *p)
{
    // 使用 HAL 库函数发送字符数据到串口
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

    // 返回输出的字符
    return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//判断外部中断源是否为GPIO_PIN_8
	if(GPIO_Pin == GPIO_PIN_8){
		switch(state){
		case wait://等待状态
			state = start;
			//检测到下降沿开始计时
			count_time_start();
			break;
		case start://起始信号和重复信息阶段
			//读取间隔时间
			count = get_tim1_cnt();
			//停止计数
			count_time_stop();
			//判断是起始信号还是重复信号
			if(count >= 13500 - 500 && count <= 13500 + 500){
				//进入数据接收阶段
				state = data_rx;
				//为数据阶段计时
				count_time_start();
			}
			else if(count >= 11250 - 500 && count <= 11250 + 500){
				//反馈重复信号，重回空闲状态
				signal_repeat = 1;
				state = wait;
			}
			else state = wait;
			break;
		case data_rx://数据处理阶段
			//读取间隔时间
			count = get_tim1_cnt();
			//停止计数
			count_time_stop();
			//数据处理
			if((count >= 1140 - 500 && count <= 1140 + 500)){
				//写入0
				ir_receive[pdata/8] &= ~(1 <<(pdata%8));
			}
			else if((count >= 2250 - 500 && count <= 2250 + 500)){
				//写入1
				ir_receive[pdata/8] |= (1 <<(pdata%8));
			}else{
				//此分支应对处理接收到的不完全数据帧和下个信号衔接恰好被识别为start信号的情况
				state = wait;
			}
			pdata++;
			//继续接收下一位byte数据
			state = data_rx;
			//数据帧处理结束
			if(pdata == 32){
				//重置pdata为0
				pdata = 0;
				//利用数据协议的反码数据校验数据准确性
				if((ir_receive[0]=~ir_receive[2])&&(ir_receive[1]=~ir_receive[3])){
					signal_data_rx_over = 1;
				}
				//重回空闲状态
				state = wait;
			}
			//数据帧未结束，为下次下降沿计时
			else count_time_start();
			break;
		}
	}
}

/* USER CODE END 0 */


void display_expression() {
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t*)expression, 12);
}

void calculate_result() {
    int operand1 = 0, operand2 = 0;
    char op = 0;
    sscanf(expression, "%d%c%d", &operand1, &op, &operand2);

    switch (op) {
        case '+': result = operand1 + operand2; break;
        case '-': result = operand1 - operand2; break;
        case '*': result = operand1 * operand2; break;
        case '/':
            if (operand2 == 0) {
                OLED_Clear();
           OLED_ShowString(0, 0, (uint8_t*)"ERR", 12);
							  for (int i = 0; i < 3; i++) {
                    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET); // ?????
                    HAL_Delay(500); // ??500??
                    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET); // ?????
                    HAL_Delay(500); // ??500??
                }
                return;
            } else {
                result = operand1 / operand2;
            }
            break;
    }
  char result_str[30]; // ?????
    sprintf(result_str, "%s=%d", expression, result); // ?????????
    OLED_Clear(); // ?? OLED ??
    OLED_ShowString(0, 0, (uint8_t*)result_str, 12); // ???????
}

void process_input(uint8_t address) {
    switch(address) {
        case 0xf3: expression[expr_len++] = '1'; break;
        case 0xe7: expression[expr_len++] = '2'; break;
        case 0xa1: expression[expr_len++] = '3'; break;
        case 0xf7: expression[expr_len++] = '4'; break;
        case 0xe3: expression[expr_len++] = '5'; break;
        case 0xa5: expression[expr_len++] = '6'; break;
        case 0xbd: expression[expr_len++] = '7'; break;
        case 0xad: expression[expr_len++] = '8'; break;
        case 0xb5: expression[expr_len++] = '9'; break;
        case 0xe9: expression[expr_len++] = '0'; break;
        case 0xea: expression[expr_len++] = '-'; break;
        case 0xf6: expression[expr_len++] = '+'; break;
        case 0xe6: expression[expr_len++] = '*'; break;
        case 0xf2: expression[expr_len++] = '/'; break;
        case 0xbb: calculate_result(); expr_len = 0; return;
        case 0xba: memset(expression, 0, sizeof(expression)); expr_len = 0; OLED_Clear(); return;
        case 0xb9: if (expr_len > 0) expression[--expr_len] = 0; break;
    }
    display_expression();
}

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
	OLED_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //对数据接收完成信号进行处理
  	  if(signal_data_rx_over == 1){
  		  //发送地址码和命令码
  		  Uart_Send_String(" address:%2x, command:%2x\r\n",ir_receive[0],ir_receive[2]);
  		     process_input(ir_receive[0]);
           signal_data_rx_over = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
