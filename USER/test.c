/*****************************************************************************

程序名称：基于STM32F103单片机的简易示波器

参考：	1）网络搜集资料、源码，整理
		2）正点原子精英版资料
			参考书：STM32F1开发指南（精英版）-寄存器版本
			部分源码：标准例程-寄存器版本
单片机型号：STM32F103ZET6

功能说明：	
	PA1   信号输入端
	PA4   信号输出端
	KEY0  切换采样频率1K、10K、100K、500K
	KEY1  切换工作状态WORK、STOP
	KEYUP 切换信号输出波形类型方波、正弦波、三角波
	
*****************************************************************************/

#include "led.h"
#include "key.h"
#include "beep.h"
#include "exti.h"
#include "timer.h"
#include "lcd.h"
#include "adc.h" 
#include "dac.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "stm32_dsp.h"
#include "table_fft.h"
#include "math.h"

#define NPT 1024   				//样点个数
#define PI2 6.28318530717959

u8 Square_table[256]={
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
};
u8 Sin_table[256]= {
	128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,
	190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,
	237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,
	255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,
	240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,
	196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,
	134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,
	62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,
	7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,
	21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,
	90,93,97,100,103,106,109,112,115,118,121,124,
};
u8 Triangle_table[256]={
	0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,
	58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,
	110,112,114,116,118,120,122,124,126,128,130,132,134,136,138,140,142,144,146,148,150,
	152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190,192,
	194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,232,234,
	236,238,240,242,244,246,248,250,252,254,254,252,250,248,246,244,242,240,238,236,234,
	232,230,228,226,224,222,220,218,216,214,212,210,208,206,204,202,200,198,196,194,192,
	190,188,186,184,182,180,178,176,174,172,170,168,166,164,162,160,158,156,154,152,150,
	148,146,144,142,140,138,136,134,132,130,128,126,124,122,120,118,116,114,112,110,108,
	106,104,102,100,98,96,94,92,90,88,86,84,82,80,78,76,74,72,70,68,66,64,62,60,58,56,
	54,52,50,48,46,44,42,40,38,36,34,32,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0,
};

u16 i=0;
u32 adcx[NPT];					//存储adc数据
u32 adcmax = 0;					//最大值
u32 adcmin = 3300;				//最小值
u16 vpp = 0;					//峰峰值变量
int long fftin [NPT];			//FFT输入
int long fftout[NPT];			//FFT输出
u16 t = 0;			//定时器中断3计数变量
u16 Set_T = 1000;	//TIM3自动重装载值
u16 Show_T = 1;		//采样率变量，单位KHZ
u32 F = 0;			//频率值
u16 U = 41;			//
u16 V = 660;		//分度单位660mv/div
u16 get_v=0;		//获得显示电压
u16 save_v=0;		//存储电压
u8 flag=0;			//采集频率标志
u8 adc_flag=0;		//采集允许标志位
u8 clear_flag=1;	//Hold on标志位，0——>Hold on
u16 table[15] ={16,32,48,64,80,96,112,128,144,160,176,192,208,224,240};//刻度X坐标数组
u16 h=0;			//数组下标
u8 Wave_flag=1;		//输出波形类型
u16 timeSet=39;		//输出波形频率

//函数声明
void lcd_huadian(u16 a,u16 b);
void lcd_huaxian(u16 x1,u16 y1,u16 x2,u16 y2);
void lcd_init(void);
void clear_point(void);		
void InitBufInArray(void);
void GetPowerMag(void);

int main(void)
 {	 
	Stm32_Clock_Init(9);		//系统时钟设置
	delay_init(72);				//延时函数初始化	  
	uart_init(72,115200);	 	//串口初始化
 	LED_Init();					//LED端口初始化
	LCD_Init();					//LCD初始化
	BEEP_Init();		 		//初始化蜂鸣器IO
	EXTIX_Init();         		//初始化外部中断输入 	 
  Adc_Init();					//ADC初始化，通道1，PA1
	Dac1_Init();				//DAC初始化，通道1，PA4
	 
	//72分频为1M,T*T1=1000		T=1000时，1kHZ		TIM3控制采样率
	TIM3_Int_Init((Set_T-1),72-1);    //(10-1,71)100kHz  (1000-1,71)1kHz	 1ms		 
	TIM6_Int_Init(timeSet,71);		    //40us
	 //初始界面
	POINT_COLOR=BLACK;
	LCD_ShowString(60,40,210,24,24,"Based on STM32F103"); 
	LCD_ShowString(62,80,200,24,24,"mini oscilloscope");
	LCD_ShowString(137,120,200,16,16,"USTC");
	LCD_ShowString(117,145,200,16,16,"SA22168274");
	LCD_ShowString(137,170,200,12,12,"2023/6");	 
	delay_ms(1000);

	LCD_Clear(WHITE);
	lcd_init(); 			//画好基本屏幕
	 
	 while(1) 
	{		 
		while(adc_flag == 0);			//等待数据采集完成，为1时说明还在采集
        adc_flag=0;			
		for(i=0;i<NPT;i++)				//NPT，1024个点，确定max与min，依次把读取的adcx值赋给fftin
		{
			fftin[i] = 0;
			fftin[i] = adcx[i] << 16;
		}
		GetPowerMag();
		adcmax=adcmax*0.80586;adcmin=adcmin*0.80586;   	//进行数模转换计算，3.3v=3300mv，0.80586 = 3300/4095，12位，2的12次方为4096
		LCD_ShowNum(275,115,adcmax,4,16);								//显示最大值
		LCD_ShowNum(275,150,adcmin,4,16);								//显示最小值

		vpp=adcmax-adcmin;
		LCD_ShowNum(275,80,vpp,4,16);				//显示峰峰值		
		clear_point();    							//更新显示屏当前列
						
		LED0=!LED0;											
		TIM3->CR1|=0x01;    						//使能定时器3			
		adcmax = 0;	adcmin = 3300;					//清除极值
//		delay_ms(1);	
	} 
}

void lcd_init(void)
{
	POINT_COLOR=RED;
	
	LCD_ShowString(15,8,200,24,24,"oscilloscope");	
	LCD_ShowString(175,15,200,16,16,"INPUT:PA1");
	LCD_ShowString(260,28,200,16,16,"mV/div:");		//毫伏/每格
	LCD_ShowString(260,63,200,16,16,"vpp/mv:");		//峰峰值/毫伏
	LCD_ShowString(260,98,200,16,16,"max/mv:");		//最大值
	LCD_ShowString(260,133,200,16,16,"min/mv:");	//最小值
	LCD_ShowString(260,168,200,16,16,"f/Hz:");		//频率/Hz
	LCD_ShowString(260,203,200,16,16,"OSR:");  		//采样率

	POINT_COLOR=BLUE;
	lcd_huaxian(0,0,0,200);				//画4条边框线
	lcd_huaxian(256,0,256,200);
	lcd_huaxian(0,0,256,0);		
	lcd_huaxian(0,200,256,200);

	POINT_COLOR=BLUE;		
	LCD_ShowString(291,220,120,16,16,"kHz");	
	LCD_ShowString(268,8,200,16,16,"WORK");

	LCD_ShowNum(265,220,Show_T,3,16);		//首次采集频率
	LCD_ShowNum(275,45,V,4,16);				//分度单位660mv/div
}

void clear_point(void)//更新显示屏当前列，主要是更新波形
{
	u16 x=0,i=0;
	
	for(x=0;x<NPT/4 && clear_flag;x++)//X轴从x=0到256逐点更新显示屏，遇到特殊坐标画线等
	{	
		POINT_COLOR=WHITE;	
		lcd_huaxian(x  ,1,x  ,199);		//删除原始线条，除上下两条边框

		POINT_COLOR=BLUE;
		lcd_huaxian(0,0,0,200);			//画左边框
		lcd_huadian(x,100);				//画中线
				
		if(x == table[h])				//当前x为刻度位，画刻度，横上、中、下线
		{
			lcd_huaxian(x,1,x,3);
			lcd_huaxian(x,101,x,103);
			lcd_huaxian(x,199,x,197);
			h++;
			if(h>=16) h=0;
		}	
		if(x==128) 						//画刻度竖中线
		{
			lcd_huaxian(x,1,x,199);
			for(i=20;i<200;i+=20)
			{
				lcd_huaxian(125,i,127,i);
			}
		}
				
		get_v = adcx[x]/U+(90-((adcmax-adcmin)/(2*U)));//计算出电压在显示屏中相对对应的y轴
		
//		lcd_huadian(x,get_v);					//画点
		lcd_huaxian(x,save_v,x+1,get_v);		//画线，方便观察，画出电压信号波形
		save_v = get_v;							//存储待用
//      delay_ms(10);	
	}
}

/******************************************************************
函数名称:GetPowerMag()
函数功能:计算各次谐波幅值
参数说明:
备　　注:先将lBufOutArray分解成实部(X)和虚部(Y)，然后计算幅值(sqrt(X*X+Y*Y)
*******************************************************************/
void GetPowerMag(void)
{
	u16 temp = 0;
    float X,Y,Mag;
	float magmax=0;
    unsigned short i;
	magmax=0;
	//调用自cr4_fft_1024_stm32
	cr4_fft_1024_stm32(fftout, fftin, NPT);	//汇编FFT库函数，1024点的FFT计算函数，输入fftin为采样点，输出fftout为幅值
    for(i=1; i<NPT/2; i++)
    {
        X  = (fftout[i] << 16) >> 16;//取低位为实部
        Y  = (fftout[i] >> 16);			 //取高位为虚部
        Mag = sqrt(X * X + Y * Y);   //求模，幅值
		if(Mag > magmax)								 //求最大幅值，记录最大值和对应点，下面根据输出频率进行转换
		{
			magmax = Mag;
			temp = i;
		}				
    }
	if(Set_T==1000)		F=(u32)((double)temp/NPT*1000  );	//计算输出频率F
	if(Set_T==100)		F=(u32)((double)temp/NPT*10010 );
	if(Set_T==10)		F=(u32)((double)temp/NPT*100200);
	if(Set_T==2)		F=(u32)((double)temp/NPT*249760);
	
	LCD_ShowNum(265,185,F,5,16);		//显示输出频率F
}

void lcd_huadian(u16 a,u16 b)			//坐标原点重定义画点
{							    
	LCD_Fast_DrawPoint(a,240-b,BLUE);
}

void lcd_huaxian(u16 x1,u16 y1,u16 x2,u16 y2)	//坐标原点重定义画线
{
	LCD_DrawLine(x1,240-y1,x2,240-y2);
}

//定时器3中断服务程序	
//每隔采样频率通过adc采样一次
void TIM3_IRQHandler(void)
{ 
	if(TIM3->SR&0X0001)//溢出中断
	{
		adcx[t]=Get_Adc(ADC_CH1);				//选择通道1(连接在PA1)，单次采样，读值给adcx[t]
//		adcx[t]=Get_Adc_Average(ADC_CH1,3);		//多次采样取均值
				
		if(adcx[t] >= adcmax)		adcmax = adcx[t];//获取极值，若当前值adcx大于已有adcmax，则赋给adcmax
		if(adcx[t] <= adcmin)		adcmin = adcx[t];
		
		t++;//1024个点中的下一个点
		if(t==NPT) //采满1024个点
		{
			t=0;
			adc_flag = 1;//表示正在采样
			TIM3->CR1 &= (uint16_t)(~((uint16_t)0x0001));//失能定时器3	
		}
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}

//外部中断3服务程序
void EXTI3_IRQHandler(void)//KEY1,暂停/继续显示屏更新，触发时反转clear_flag和采样定时器3
{
	delay_ms(10);	//消抖
	if(KEY1==0)	 	//按键KEY1
	{	
		if(clear_flag == 1)
		{
			TIM3->CR1 &= (uint16_t)(~((uint16_t)0x0001));//失能定时器3					
			clear_flag=0;
			LCD_ShowString(268,8,200,16,16,"STOP  ");			
		}
		else
		{
			TIM3->CR1|=0x01;    //使能定时器3				
			clear_flag=1;
			LCD_ShowString(268,8,200,16,16,"WORK");
		}
		while(KEY1==0);
	}		 
	EXTI->PR=1<<3;  //清除LINE3上的中断标志位  
}
//外部中断4服务程序
void EXTI4_IRQHandler(void)//KEY0,更改Set_T以改变采样频率，并更新对应的show_T值显示于OSR
{
	delay_ms(10);	//消抖
	if(KEY0==0)	 	//按键KEY0
	{
		POINT_COLOR=BLUE;		
		flag++;
		if(flag==1)
		{
			Set_T = 100;
			Show_T = 10;
		}
		else if(flag==2) 
		{
			Set_T=10;
			Show_T = 100;
		}
		else if(flag==3)
		{
			Set_T=2;
			Show_T = 500;
		}
		else if(flag==4)
		{
			Set_T=1000;
			Show_T = 1;
			flag=0;
		}
		LCD_ShowNum(265,220,Show_T,3,16);						   
		TIM3->ARR=(Set_T-1);  			//设定计数器自动重装值，更新采样频率
		while(KEY0==0);							//若按住则一直停留
	}
	EXTI->PR=1<<4;  					//清除LINE4上的中断标志位  
}		   

//定时器6中断服务程序	 
void TIM6_IRQHandler(void)//DAC模块根据Wave_flag选择预设波形输出信号
{ 	
	static u16 intr=0;
	if(TIM6->SR&0X0001)		//溢出中断
	{
		intr++;
		if(intr==256)
			intr=0;
		switch (Wave_flag)
		{
			case 1:Dac1_Set_Vol(Square_table[intr]*12);		//DA输出方波数组电压，乘以12位
				break;
			case 2:Dac1_Set_Vol(Sin_table[intr]*12);		//DA输出正弦波数组电压
				break;
			case 3:Dac1_Set_Vol(Triangle_table[intr]*12);	//DA输出三角波数组电压
				break;
			default :
				break;
		}
	}				   
	TIM6->SR&=~(1<<0);	//清除中断标志位 	    
}

//外部中断0服务程序
void EXTI0_IRQHandler(void)//WK_UP,改变Wave_flag标志位
{
	delay_ms(10);	//消抖
	if(WK_UP==1)	//WK_UP按键
	{				 
		if(Wave_flag==1)
			Wave_flag=2;
		else if(Wave_flag==2)
			Wave_flag=3;
		else if(Wave_flag==3)
			Wave_flag=1;		
		while(WK_UP==1);
	}		 
	EXTI->PR=1<<0;  //清除LINE0上的中断标志位  
}
