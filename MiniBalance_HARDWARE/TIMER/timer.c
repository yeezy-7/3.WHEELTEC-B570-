/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��5.7
�޸�ʱ�䣺2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update��2021-04-29

All rights reserved
***********************************************/
#include "timer.h"
/**************************************************************************
Function: Timer 2 channel 2 input capture initialization
Input   : arr��Auto reload value�� psc�� Clock prescaled frequency
Output  : none
�������ܣ���ʱ��2ͨ��2���벶���ʼ��
��ڲ���: arr���Զ���װֵ�� psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/	 		
TIM_ICInitTypeDef  TIM2_ICInitStructure;
void TIM2_Cap_Init(u16 arr,u16 psc)	
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef TIM_OCInitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//ʹ��TIM2ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA1 ����  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //PA3��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;     
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     //PA0��� 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//��ʼ����ʱ��2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
//  
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             	//ѡ��PWM1ģʽ
//	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable; //�Ƚ����ʹ��
////	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
//	TIM_OCInitStructure.TIM_Pulse = 0;                            	//���ô�װ�벶��ȽϼĴ���������ֵ
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     	//�����������
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//	
//	TIM_OC1Init(TIM2,&TIM_OCInitStructure);         //��ʼ������Ƚϲ�����ͨ��3

//	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);  	//CH1ʹ��Ԥװ�ؼĴ���
//	TIM_ARRPreloadConfig(TIM2, ENABLE);               			 	//ʹ��TIM3��ARR�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM2���벶�����
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=02 	ѡ������� IC2ӳ�䵽TI1��
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;//���������˲��� ���˲�
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC2IE�����ж�	
    TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��2
//	TIM2->CCR1 = 1500;
}
/**************************************************************************
Function: Ultrasonic receiving echo function
Input   : none
Output  : none
�������ܣ����������ջز�����
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
u16 TIM2CH2_CAPTURE_STA,TIM2CH2_CAPTURE_VAL;
void Read_Distane(void)        
{   
	 PAout(3)=1;         
	 delay_us(15);  
	 PAout(3)=0;	
	 if(TIM2CH2_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
	 {
		 Distance=TIM2CH2_CAPTURE_STA&0X3F; 
		 Distance*=65536;					        //���ʱ���ܺ�
		 Distance+=TIM2CH2_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
		 Distance=Distance*170/1000;      //ʱ��*����/2�����أ� һ������0.001ms
		 TIM2CH2_CAPTURE_STA=0;			//������һ�β���
	 }				
}
/**************************************************************************
Function: Pulse width reading interruption of ultrasonic echo
Input   : none
Output  : none
�������ܣ��������ز������ȡ�ж�
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
void TIM2_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM2->SR;
	if((TIM2CH2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(tsr&0X01)//��ʱ�����
		{	    
			 if(TIM2CH2_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			 {
				 if((TIM2CH2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				 {
					  TIM2CH2_CAPTURE_STA|=0X80;      //��ǳɹ�������һ��
						TIM2CH2_CAPTURE_VAL=0XFFFF;
				 }else TIM2CH2_CAPTURE_STA++;
			 }	 
		}
		if(tsr&0x04)//����2���������¼�
		{	
			if(TIM2CH2_CAPTURE_STA&0X40)		  //����һ���½��� 		
			{	  	
         	    
				TIM2CH2_CAPTURE_STA|=0X80;		  //��ǳɹ�����һ�θߵ�ƽ����
				TIM2CH2_CAPTURE_VAL=TIM2->CCR2;	//��ȡ��ǰ�Ĳ���ֵ.
				TIM2->CCER&=~(1<<5);			      //CC2P=0 ����Ϊ�����ز���
			}
			else  								     //��δ��ʼ,��һ�β���������
			{
				 TIM2CH2_CAPTURE_STA=0;	 //���
				 TIM2CH2_CAPTURE_VAL=0;
				 TIM2CH2_CAPTURE_STA|=0X40;		//��ǲ�����������
				 TIM2->CNT=0;									//���������
				 TIM2->CCER|=1<<5; 						//CC2P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
	}
	TIM2->SR=0;//����жϱ�־λ 	     
}
