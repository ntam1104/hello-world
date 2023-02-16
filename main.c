#include "stm32f10x.h"
#include "nrf24_drive.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

//#define MODE_NRF24 1  // MODE_NRF24 = 0 --> TX MODE | MODE_NRF24 = 1 --> RX MODE

//char payload[32]= {0};
//char payload_id[32]= {0};
//uint8_t send[32] = {0};
char SpeedOfMotor [4] = {0};
char SetSpeed [4] = {0};
char AngleOfServo [4] = {0};
char SetAngle [4] = {0};
char rx[32] = {0};
int count = 0;
void USART3_Init(void);
int num = 0, num1 = 0, num2 = 0, num3 = 0;
int btn_last = 0, btn_last1 = 0;
int servo = 0, servo1 = 0, setServo = 0;
void PWM_Init(void);
void PWM_Init1(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
int pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;
int speed = 0, angle = 0, setAngle = 0, setSpeed = 0;
int start = 1;
int maxSpeed = 0;
void GPIO_Configuration(void);

int main(void)
{
	GPIO_Configuration();
	PWM_Init();
	PWM_Init1();
	USART3_Init();
//	nrf_init(1,MODE0);
//	
//	// PRX Mode
//	if(MODE_NRF24 == 1)
//	{
//		nrf_prx_init(1);
//	}
//	Digital_Output(PC,13);
//	//Digital_Output(PA,0);
//	//Digital_Input(PA,1);
//	while(1)
//	{
////		adcVal[0] = ADC1_Read(ADC_Channel_4);
////		adcVal[1] = ADC1_Read(ADC_Channel_5);
//		
//			// PRX Mode
//		if(MODE_NRF24 == 1)
//		{
//			//Recieving Code
//			if(nrf_msg_check(1))
//			{
//				nrf_rx(1,payload);
//				if( payload[0] == 'T' && payload[1] == 'A'  && payload[2] == 'M')
//				{
//					for(int i = 0; i < 32; i++)
//					payload_id[i] = payload[i];
//					toggle_GP(PC,13);
//					
//					
//					arr[0] = payload_id[5];
//					arr[1] = payload_id[4];
//					arr[2] = payload_id[3];
//					sscanf(arr, "%d", &num);
//					
//					arr1[0] = payload_id[8];
//					arr1[1] = payload_id[7];
//					arr1[2] = payload_id[6];
//					sscanf(arr1, "%d", &num1);
//					
//					if (payload_id[9] != btn_last)
//					{
//						 btn_last = payload_id[9];
//					}
//					
//					
//					arr2[0] = payload_id[12];
//					arr2[1] = payload_id[11];
//					arr2[2] = payload_id[10];
//					sscanf(arr2, "%d", &num2);
//					
//					arr3[0] = payload_id[15];
//					arr3[1] = payload_id[14];
//					arr3[2] = payload_id[13];
//					sscanf(arr3, "%d", &num3);
//					
//					if (payload_id[16] != btn_last1)
//					{
//						 btn_last1 = payload_id[16];
//					}
//					
//				}
//		}
	while(1)
	{
		if(rx[0] == 'T' && rx[1] == 'A' && rx[2] == 'M')
		{
			SpeedOfMotor[0] = rx[3];
			SpeedOfMotor[1] = rx[4];
			SpeedOfMotor[2] = rx[5];
			speed = atoi(SpeedOfMotor);
			
			
			
			AngleOfServo[0] = rx[7];
			AngleOfServo[1] = rx[8];
			AngleOfServo[2] = rx[9];
			angle = atoi(AngleOfServo);
			
			SetSpeed[0] = rx[14];
			SetSpeed[1] = rx[15];
			SetSpeed[2] = rx[16];
			setSpeed = atoi(SetSpeed);
			maxSpeed = map(setSpeed, 0, 100, 0, 7199);
			
			
			
			SetAngle[0] = rx[11];
			SetAngle[1] = rx[12];
			SetAngle[2] = rx[13];
			setAngle = atoi(SetAngle);
				
				if (speed < 52 && speed > 48)
				{
					pwm1 = 0;
					pwm2 = 0;
					//GPIO_ResetBits(GPIOA, GPIO_Pin_8);
					//GPIO_ResetBits(GPIOA, GPIO_Pin_9);
				}	
				else if (speed > 52 && rx[6] == 'u')
				{
					pwm1 = (speed-49)*2*maxSpeed/100;
					pwm2 = 0;
					TIM4 -> CCR1 = pwm1;
	//				GPIO_SetBits(GPIOA, GPIO_Pin_8);
//					GPIO_ResetBits(GPIOA, GPIO_Pin_9);
				}
				else
				{
					int x = 100 - (speed*2);
					pwm1 = 0;
					pwm2 = x*maxSpeed/100;
					TIM4 -> CCR2 = pwm2;
//				GPIO_ResetBits(GPIOA, GPIO_Pin_8);
//				GPIO_SetBits(GPIOA, GPIO_Pin_9);
				}
			
	//Set goc	
				if(start == 1)
				{
					setServo = map(setAngle,0,100,750,1250);
					TIM3 -> CCR4 = setServo;
					if(pwm1 != 0)
					{
						start = 0;
					}
				}
				else
					{
					if (angle < 52 && angle > 48)
					{
						servo = 0;
						servo1 = 0;
						TIM3->CCR4 = setServo;
					}
					else if (angle > 52 && rx[10] == 'r')
					{
						servo1 = map(angle,52,100,setServo,750);
						TIM3 -> CCR4 = servo1;
					}
					else 
					{
						servo = map(angle,0,49,1250,setServo);
						TIM3 -> CCR4 = servo;
					}
				}
			
		}
	}
}
//}


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//////void ADC1_Init()
//////{
//////	// Initialization struct
//////	ADC_InitTypeDef ADC_InitStruct;
//////	GPIO_InitTypeDef GPIO_InitStruct;
//////	
//////	// Step 1: Initialize ADC1
//////	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//////	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
//////	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
//////	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
//////	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//////	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
//////	ADC_InitStruct.ADC_NbrOfChannel = 2;
//////	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
//////	ADC_Init(ADC1, &ADC_InitStruct);
//////	ADC_Cmd(ADC1, ENABLE);
//////	// Select input channel for ADC1
//////	// ADC1 channel 0 (PA0)
//////	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
//////	
//////	// Step 2: Initialize GPIOA (PA0)
//////	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//////	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
//////	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
//////	GPIO_Init(GPIOA, &GPIO_InitStruct);
//////	
//////}

//////uint16_t ADC1_Read(uint8_t channel)
//////{
//////	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
//////	// Start ADC conversion
//////	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//////	// Wait until ADC conversion finished
//////	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

//////	return ADC_GetConversionValue(ADC1);
//////}

void PWM_Init()
{
	// Initialization struct
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// Step 1: Initialize TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// Create 1kHz PWM
	// TIM2 is connected to APB1 bus that have default clock 72MHz
	// So, the frequency of TIM2 is 72MHz
	// We use prescaler 10 here
	// So, the frequency of TIM2 now is 72MHz
	TIM_TimeBaseInitStruct.TIM_Prescaler = 100;
	// TIM_Period determine the PWM frequency by this equation:
	// PWM_frequency = timer_clock / (TIM_Period + 1)
	// If we want 1kHz PWM we can calculate:
	// TIM_Period = (timer_clock / PWM_frequency) - 1
	// TIM_Period = (7.2MHz / 1kHz) - 1 = 7199
	TIM_TimeBaseInitStruct.TIM_Period = 7199;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	// Start TIM2
	TIM_Cmd(TIM4, ENABLE);
	
	// Step 2: Initialize PWM
	// Common PWM settings
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	// Duty cycle calculation equation:
	// TIM_Pulse = (((TIM_Period + 1) * duty_cycle) / 100) - 1
	// Ex. 25% duty cycle:
	// 		TIM_Pulse = (((7199 + 1) * 25) / 100) - 1 = 1799
	//		TIM_Pulse = (((7199 + 1) * 75) / 100) - 1 = 5399
	// We initialize PWM value with duty cycle of 0%
	TIM_OCInitStruct.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStruct);
	TIM_OC2Init(TIM4, &TIM_OCInitStruct);
	TIM_OC3Init(TIM4, &TIM_OCInitStruct);
	TIM_OC4Init(TIM4, &TIM_OCInitStruct);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	// Step 3: Initialize GPIOA (PA0)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// Initialize PA0 as push-pull alternate function (PWM output) for LED
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
}

void USART3_IRQHandler()
{
	// Check if the USART3 receive interrupt flag was set
	if (USART_GetITStatus(USART3, USART_IT_RXNE))
	{
		rx[count] = USART_ReceiveData(USART3);
		
		if(rx[count] == '\n')
		{
			count = 0;
		}
		else
		{
			count++;
		}
		
	}
}

void USART3_Init()
{
	// Initialization struct
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	// Step 1: Initialize USART3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStruct);
	USART_Cmd(USART3, ENABLE);
	
	// Step 2: Initialize GPIO for Tx and Rx pin
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// Tx pin (PA2) initialization as push-pull alternate function
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	// Rx pin (PA3) initialization as input floating
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// Step 3: Enable USART receive interrupt
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	// Step 4: Initialize NVIC for USART IRQ
	// Set NVIC prority group to group 4 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	// Set System Timer IRQ at higher priority
	NVIC_SetPriority(SysTick_IRQn, 0);
	// Set USART3 IRQ at lower priority
	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}



//void PWM_Init()
//{
//	// Initialization struct
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
//	TIM_OCInitTypeDef TIM_OCInitStruct;
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	// Step 1: Initialize TIM2
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
//	// Create 1kHz PWM
//	// TIM2 is connected to APB1 bus that have default clock 72MHz
//	// So, the frequency of TIM2 is 72MHz
//	// We use prescaler 10 here
//	// So, the frequency of TIM2 now is 72MHz
//	TIM_TimeBaseInitStruct.TIM_Prescaler = 10;
//	// TIM_Period determine the PWM frequency by this equation:
//	// PWM_frequency = timer_clock / (TIM_Period + 1)
//	// If we want 1kHz PWM we can calculate:
//	// TIM_Period = (timer_clock / PWM_frequency) - 1
//	// TIM_Period = (7.2MHz / 1kHz) - 1 = 7199
//	TIM_TimeBaseInitStruct.TIM_Period = 7199;
//	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
//	// Start TIM2
//	TIM_Cmd(TIM4, ENABLE);
//	
//	// Step 2: Initialize PWM
//	// Common PWM settings
//	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
//	// Duty cycle calculation equation:
//	// TIM_Pulse = (((TIM_Period + 1) * duty_cycle) / 100) - 1
//	// Ex. 25% duty cycle:
//	// 		TIM_Pulse = (((7199 + 1) * 25) / 100) - 1 = 1799
//	//		TIM_Pulse = (((7199 + 1) * 75) / 100) - 1 = 5399
//	// We initialize PWM value with duty cycle of 0%
//	TIM_OCInitStruct.TIM_Pulse = 0;
//	
//	TIM_OC1Init(TIM4, &TIM_OCInitStruct);
//	TIM_OC2Init(TIM4, &TIM_OCInitStruct);
//	TIM_OC3Init(TIM4, &TIM_OCInitStruct);
//	TIM_OC4Init(TIM4, &TIM_OCInitStruct);

//	
//	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	
//	
//	// Step 3: Initialize GPIOA (PA0)
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	// Initialize PA0 as push-pull alternate function (PWM output) for LED
//	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
//}

void PWM_Init1() 
{
	// Initialization struct
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// Step 1: Initialize TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// Create 50Hz PWM
	// Prescale timer clock from 72MHz to 720kHz by prescaler = 100
	TIM_TimeBaseInitStruct.TIM_Prescaler = 100;
	// TIM_Period = (timer_clock / PWM_frequency) - 1
	// TIM_Period = (720kHz / 50Hz) - 1 = 14399
	TIM_TimeBaseInitStruct.TIM_Period = 14399;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	// Start TIM2
	TIM_Cmd(TIM3, ENABLE);
	
	// Step 2: Initialize PWM
	// Common PWM settings
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	// We initialize PWM value with duty cycle of 0%
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	// Step 3: Initialize GPIOA (PA0)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// Initialize PA0 as push-pull alternate function (PWM output) for LED
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void GPIO_Configuration(void)
{
GPIO_InitTypeDef GPIO_InitStructure;


RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 ;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init(GPIOA, &GPIO_InitStructure);
}

