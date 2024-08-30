#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "key.h"
#include "beep.h"

//�ⲿ�жϳ�ʼ������
//��ʼ��PA0/PE2/PE3/PE4Ϊ�ж�����.
void EXTIX_Init(void)
{
	KEY_Init();
	Ex_NVIC_Config(GPIO_A,0,RTIR); 	//�����ش���
	Ex_NVIC_Config(GPIO_E,2,FTIR); 	//�½��ش���
	Ex_NVIC_Config(GPIO_E,3,FTIR); 	//�½��ش���
 	Ex_NVIC_Config(GPIO_E,4,FTIR); 	//�½��ش���
	MY_NVIC_Init(2,3,EXTI0_IRQn,2);	//��ռ2�������ȼ�3����2
	MY_NVIC_Init(2,2,EXTI2_IRQn,2);	//��ռ2�������ȼ�2����2	   
	MY_NVIC_Init(2,1,EXTI3_IRQn,2);	//��ռ2�������ȼ�1����2	   
	MY_NVIC_Init(2,0,EXTI4_IRQn,2);	//��ռ2�������ȼ�0����2	   
}

