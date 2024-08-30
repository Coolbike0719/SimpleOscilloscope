#include "timer.h"
#include "led.h"

//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr;  	//设定计数器自动重装值 
	TIM3->PSC=psc;  	//预分频器设置
	TIM3->DIER|=1<<0;   //允许更新中断				
	TIM3->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(2,4,TIM3_IRQn,2);//抢占1，子优先级3，组2									 
}
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器6
void TIM6_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<4;	//TIM6时钟使能    
 	TIM6->ARR=arr;  	//设定计数器自动重装值 
	TIM6->PSC=psc;  	//预分频器设置
	TIM6->DIER|=1<<0;   //允许更新中断				
	TIM6->CR1|=0x01;    //使能定时器6
  	MY_NVIC_Init(2,3,TIM6_IRQn,2);//抢占1，子优先级3，组2									 
}

