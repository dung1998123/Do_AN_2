#include "main.h"							//UART1 DOC TU GPS 
#include "lcd_txt.h" 					//UART5 DOC TU WIFI VA GUI QUA WIFI MOI 10S  // UART4 DOC TU GPS
#include "stdio.h"

#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0

#define Sampling_time		25	 //thoi gian lay mau (ms)
#define inv_Sampling_time	40 	 // 1/Sampling_time 
#define PWM_Period			8400 //8400 cycles = 1ms


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);

void hien_thi();
void delay_us(int us);
void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data);
uint32_t Flash_Read(uint32_t Flash_Address);
void Motor_Speed_PID(long int des_Speed, int dc);
void tien();
void lui();
void q_trai();
void q_phai();
void stop();
void send_laban(uint8_t lb);
void quay_goc(int gry);
void theo_y();
void control();

volatile long int Pulse1, Pre_Pulse1, Pulse2, Pre_Pulse2;
volatile long int rSpeed1, Err1, Pre_Err1, Kp=10, Kd=10, Ki=1;
volatile long int rSpeed2, Err2, Pre_Err2;
volatile long int Output1, Output2;
volatile unsigned char Sample_Count=0;
volatile	long int pPart1=0, iPart1=0, dPart1=0; //PID gains
volatile	long int pPart2=0, iPart2=0, dPart2=0; //PID gains
volatile int A=0,B=0;
uint8_t byte[2],toado[2],toado1[2],gui[4];
uint8_t a,b;
int goc,goc1;
char buff[20];


int main(void)
{
  
  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
	
	HAL_UART_Receive_IT(&huart5,&byte[0],2);
	HAL_UART_Receive_IT(&huart1,&toado1[0],2);
	HAL_UART_Receive_IT(&huart4,&toado[0],2);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		control();
  }
  /* USER CODE END 3 */
}



void delay_us(int us)
{
	int i;
	int j;
	for(i=0;i<=us;i++){
	for(j=0;j<us;j++){}
}	
}


void Flash_Write(uint32_t Flash_Address, uint32_t Flash_Data)  						//ghi flash
{
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_3);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Flash_Address,Flash_Data);
	HAL_FLASH_Lock();
}

uint32_t Flash_Read(uint32_t Flash_Address) 															//doc flash
{
	uint32_t Flash_Data;
	
	Flash_Data = *(uint32_t*) Flash_Address;
	return Flash_Data;
}

void Motor_Speed_PID(long int des_Speed, int dc)													//tinh pid cho motor
{
	if(dc==1)    //dong co 1
	{
		rSpeed1=Pulse1-Pre_Pulse1;     //tinh van toc (trong sampling time)
		Pre_Pulse1=Pulse1;            //luu lai gia tri Pulse: so xung 
		Err1=des_Speed-rSpeed1;  //tinh error (loi)
    //cac thanh phan cua PID
		pPart1=Kp*Err1;
		dPart1=Kd*(Err1-Pre_Err1)*inv_Sampling_time;	
		iPart1+=Ki*Sampling_time*Err1/1000;
		Output1 +=pPart1+dPart1+iPart1;     //cong thuc duoc bien doi vi la dieu khien van toc
    //saturation
		if (Output1 >=PWM_Period) Output1=PWM_Period-1;
		if (Output1 <=0) Output1=1;
    
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Output1);
		Pre_Err1=Err1;  //luu lai gia tri error  
	}
	if(dc==2)	//dong co 2
	{
		rSpeed2=Pulse2-Pre_Pulse2;     //tinh van toc (trong sampling time)
		Pre_Pulse2=Pulse2;            //luu lai gia tri Pulse: so xung 
		Err2=des_Speed-rSpeed2;  //tinh error (loi)
    //cac thanh phan cua PID
		pPart2=Kp*Err2;
		dPart2=Kd*(Err2-Pre_Err2)*inv_Sampling_time;	
		iPart2+=Ki*Sampling_time*Err2/1000;
		Output2 +=pPart2+dPart2+iPart2;     //cong thuc duoc bien doi vi la dieu khien van toc
    //saturation
		if (Output2 >=PWM_Period) Output2=PWM_Period-1;
		if (Output2 <=0) Output2=1;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Output2);
		Pre_Err2=Err2;  //luu lai gia tri error  
	}
}	

void tien()
{

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
	A=84;
	B=84;	
	hien_thi();
}

void lui()
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
	A=84;
	B=84;	
	hien_thi();
}

void q_trai()
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
	A=84;
	B=84;	
	hien_thi();
}

void q_phai()
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
	A=84;
	B=84;	
	hien_thi();
}

void stop()
{
	
	A=B=0;

}


void send_laban(uint8_t lb)	 					//gui uart cho laban
{
	a = lb;
	HAL_UART_Transmit_IT(&huart5,&a,1);
}

void quay_goc(int gry)  //quay theo huong di
{
	goc1=gry;
	send_laban('z');
	if(goc1 > goc)
	{
		while(1)
		{
			send_laban('z');
			q_trai();
			if(goc1 < goc)
			{
				stop();
				break;
			}
			
		}
	}
	if(goc1 < goc)
	{
		while(1)
		{
			send_laban('z');
			q_phai();
			if(goc1 > goc)
			{
				stop();
				break;
			}
		}
	}
}

void control()
{
	if(toado[0] < toado1[0])
	{
		quay_goc(180);
		tien();
		if(toado[0] >= toado1[0])
		theo_y();
	}
	if(toado[0] > toado1[0])
	{
		quay_goc(0);
		tien();
		if(toado[0] <= toado1[0])
		theo_y();
	}
	if(toado[0] == toado1[0])
	{
		theo_y();
	}
}
void theo_y()
{
	if(toado[1] < toado1[1])
	{
		quay_goc(90);
		tien();
		if(toado[1] >= toado1[1])
			stop();
	}
	if(toado[1] > toado1[1])
	{
		quay_goc(270);
		tien();
		if(toado[1] <= toado1[1])
			stop();
	}
}
	


void hien_thi()				//hien thi so xung de chinh pid
{
	int i,j;
	for(i=0; i<4; i++)
	{
	for(j=0; j<5; j++)
		{
			sprintf(buff, "%3ld", rSpeed1);
			lcd_puts(i,4*j,(int8_t*)buff);
			HAL_Delay(500);
		}	
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)		//ngat ngoai doc encoder
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		Pulse1++;
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0));
		
	}
	if(GPIO_Pin == GPIO_PIN_1)
	{
		Pulse2++;
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1));
		
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  		//ngat tran timer4 sampling time   //gui toa do vao flash va uart timer2
{
	if(htim->Instance==htim4.Instance)    //ngat tran timer4 25ms=sampling time
	{
		Motor_Speed_PID(A,1);
		Motor_Speed_PID(B,2);
		Sample_Count++;
	}
	if(htim->Instance==htim2.Instance)
	{
		//dien code gui x,y qua uart1// moi 0,1s
		gui[0] = toado[0];
		gui[1] = toado[1];
		gui[2] = rSpeed1;
		gui[3] = rSpeed2;
		HAL_UART_Transmit_IT(&huart5,&gui[0],4);	//gui 2byte dau la toado[] 2 byte sau la xung;
		Flash_Write(0x080E0000,(toado[0]<<8)|(toado[1]));  //gui toa do vao bo nho flash
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)		//nhan goc laban, toa do den, toa do hien tai
{
	if(huart->Instance==huart5.Instance)				//nhan goc laban
	{
		HAL_UART_Receive_IT(&huart5,&byte[0],2);
		goc = (byte[0]<<8)|byte[1];
		if(goc>30000)
		{
			goc=goc-65553;
		}
		goc=goc/10+90;
	
		}
	if(huart->Instance==huart1.Instance)				//nhan toa do den 
	{
		HAL_UART_Receive_IT(&huart1,&toado1[0],2);
	}
	
	if(huart->Instance==huart4.Instance)				//nhan toa do qua gps
	{
		HAL_UART_Receive_IT(&huart4,&toado[0],2);
	}
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

 
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_TIM1_Init(void)
{

  

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};


  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_TIM_MspPostInit(&htim1);

}


static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  

}


static void MX_TIM4_Init(void)
{

  
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  

}


static void MX_UART4_Init(void)
{

 
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  
}


static void MX_UART5_Init(void)
{

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
 
}


static void MX_USART1_UART_Init(void)
{

  
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  

}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE8 PE10 PE12 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	/* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);


}


  
void Error_Handler(void)
{
  
}


#ifdef  USE_FULL_ASSERT


void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
