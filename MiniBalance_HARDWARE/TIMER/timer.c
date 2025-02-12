/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "timer.h"
/**************************************************************************
Function: Timer 2 channel 2 input capture initialization
Input   : arr：Auto reload value； psc： Clock prescaled frequency
Output  : none
函数功能：定时器2通道2输入捕获初始化
入口参数: arr：自动重装值； psc：时钟预分频数 
返回  值：无
**************************************************************************/	 		
TIM_ICInitTypeDef  TIM2_ICInitStructure;
void TIM2_Cap_Init(u16 arr,u16 psc)	
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef TIM_OCInitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能TIM2时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA1 输入  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //PA3输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;     
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     //PA0输出 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//初始化定时器2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
//  
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             	//选择PWM1模式
//	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable; //比较输出使能
////	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
//	TIM_OCInitStructure.TIM_Pulse = 0;                            	//设置待装入捕获比较寄存器的脉冲值
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     	//设置输出极性
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//	
//	TIM_OC1Init(TIM2,&TIM_OCInitStructure);         //初始化输出比较参数，通道3

//	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);  	//CH1使能预装载寄存器
//	TIM_ARRPreloadConfig(TIM2, ENABLE);               			 	//使能TIM3在ARR上的预装载寄存器
	
	//初始化TIM2输入捕获参数
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=02 	选择输入端 IC2映射到TI1上
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;//配置输入滤波器 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC2IE捕获中断	
    TIM_Cmd(TIM2,ENABLE ); 	//使能定时器2
//	TIM2->CCR1 = 1500;
}
/**************************************************************************
Function: Ultrasonic receiving echo function
Input   : none
Output  : none
函数功能：超声波接收回波函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
u16 TIM2CH2_CAPTURE_STA,TIM2CH2_CAPTURE_VAL;
void Read_Distane(void)        
{   
	 PAout(3)=1;         
	 delay_us(15);  
	 PAout(3)=0;	
	 if(TIM2CH2_CAPTURE_STA&0X80)//成功捕获到了一次高电平
	 {
		 Distance=TIM2CH2_CAPTURE_STA&0X3F; 
		 Distance*=65536;					        //溢出时间总和
		 Distance+=TIM2CH2_CAPTURE_VAL;		//得到总的高电平时间
		 Distance=Distance*170/1000;      //时间*声速/2（来回） 一个计数0.001ms
		 TIM2CH2_CAPTURE_STA=0;			//开启下一次捕获
	 }				
}
/**************************************************************************
Function: Pulse width reading interruption of ultrasonic echo
Input   : none
Output  : none
函数功能：超声波回波脉宽读取中断
入口参数: 无 
返回  值：无
**************************************************************************/	 
void TIM2_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM2->SR;
	if((TIM2CH2_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(tsr&0X01)//定时器溢出
		{	    
			 if(TIM2CH2_CAPTURE_STA&0X40)//已经捕获到高电平了
			 {
				 if((TIM2CH2_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				 {
					  TIM2CH2_CAPTURE_STA|=0X80;      //标记成功捕获了一次
						TIM2CH2_CAPTURE_VAL=0XFFFF;
				 }else TIM2CH2_CAPTURE_STA++;
			 }	 
		}
		if(tsr&0x04)//捕获2发生捕获事件
		{	
			if(TIM2CH2_CAPTURE_STA&0X40)		  //捕获到一个下降沿 		
			{	  	
         	    
				TIM2CH2_CAPTURE_STA|=0X80;		  //标记成功捕获到一次高电平脉宽
				TIM2CH2_CAPTURE_VAL=TIM2->CCR2;	//获取当前的捕获值.
				TIM2->CCER&=~(1<<5);			      //CC2P=0 设置为上升沿捕获
			}
			else  								     //还未开始,第一次捕获上升沿
			{
				 TIM2CH2_CAPTURE_STA=0;	 //清空
				 TIM2CH2_CAPTURE_VAL=0;
				 TIM2CH2_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				 TIM2->CNT=0;									//计数器清空
				 TIM2->CCER|=1<<5; 						//CC2P=1 设置为下降沿捕获
			}		    
		}			     	    					   
	}
	TIM2->SR=0;//清除中断标志位 	     
}
