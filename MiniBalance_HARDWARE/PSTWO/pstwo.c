/***********************************************
��˾����Ȥ�Ƽ�(��ݸ)���޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2022-09-05

All rights reserved
***********************************************/

#include "./PSTWO/pstwo.h"
#define DELAY_TIME  delay_us(5); 
u16 Handkey;	// ����ֵ��ȡ����ʱ�洢��
u8 Comd[2]={0x01,0x42};	//��ʼ�����������
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//����ֵ�밴����

//The PS2 gamepad controls related variables
//PS2�ֱ�������ر���
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

/**************************************************************************
Function: PS2_Init
Input   : none
Output  : none
��������: PS2�ֱ���ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
	
void PS2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(PS2_DI_GPIO_CLK|PS2_DO_GPIO_CLK,ENABLE);     //ʹ��ʱ��,DI,DO 
	RCC_APB2PeriphClockCmd(PS2_CS_GPIO_CLK|PS2_CLK_GPIO_CLK,ENABLE);    //ʹ��ʱ��,CS,CLK
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;           				//��������
	GPIO_InitStructure.GPIO_Pin=PS2_DI_GPIO_PIN;                 		// DI 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(PS2_DI_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;              		//ͨ���������
	GPIO_InitStructure.GPIO_Pin=PS2_DO_GPIO_PIN;                 		// DO 
	GPIO_Init(PS2_DO_GPIO_PORT,&GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;           			//ͨ���������
	GPIO_InitStructure.GPIO_Pin=PS2_CS_GPIO_PIN;     					// CS  
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(PS2_CS_GPIO_PORT,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;           			//ͨ���������
	GPIO_InitStructure.GPIO_Pin=PS2_CLK_GPIO_PIN;     					// CLK  
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(PS2_CLK_GPIO_PORT,&GPIO_InitStructure);

	
//	PWR->CR|=1<<8;	//ȡ��������д���� 
//	RCC->BDCR&=0xFFFFFFFE;	//�ⲿ���������ر� PC14��PC15��Ϊ��ͨIO	 	
//	BKP->CR&=0xFFFFFFFE; 	//������TAMPER������Ϊͨ��IO��ʹ�� 
//	PWR->CR&=0xFFFFFEFF;	//������д����	
}
/**************************************************************************
Function: Sent_Cmd
Input   : none
Output  : none
��������: ���ֱ���������
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//���ֱ���������
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                  	 //���һλ����λ
		}
		else DO_L;

		CLK_H;                        //ʱ������
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	delay_us(16);
}
//�ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
//����ֵ��0�����ģʽ
//		  ����������ģʽ
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//��ȡ�ֱ�����
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	for(byte=2;byte<9;byte++)          //��ʼ��������
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}

//�Զ�������PS2�����ݽ��д���,ֻ����������  
//ֻ��һ����������ʱ����Ϊ0�� δ����Ϊ1
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //û���κΰ�������
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	   motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
    PS2_Cmd(0x01);  //��ʼ����
	PS2_Cmd(0x42);  //��������
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
//short poll
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
//��������
void PS2_EnterConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  ������÷���ģʽ
	PS2_Cmd(0x03); //Ox03�������ã�������ͨ��������MODE������ģʽ��
				   //0xEE������������ã���ͨ��������MODE������ģʽ��
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//������
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
//��ɲ���������
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//��������ģʽ
	PS2_TurnOnAnalogMode();	//�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	//PS2_VibrationMode();	//������ģʽ
	PS2_ExitConfing();		//��ɲ���������
}



/**************************************************************************
Function: Read the control of the ps2 handle
Input   : none
Output  : none
�������ܣ���ȡPS2�ֱ��Ŀ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_Read(void)
{
	static int Strat;
	//Reading key
  //��ȡ������ֵ
	PS2_KEY=PS2_DataKey(); 
	//Read the analog of the remote sensing x axis on the left
  //��ȡ���ң��X�᷽���ģ����	
	PS2_LX=PS2_AnologData(PSS_LX); 
	//Read the analog of the directional direction of remote sensing on the left
  //��ȡ���ң��Y�᷽���ģ����	
	PS2_LY=PS2_AnologData(PSS_LY);
	//Read the analog of the remote sensing x axis
  //��ȡ�ұ�ң��X�᷽���ģ����  
	PS2_RX=PS2_AnologData(PSS_RX);
	//Read the analog of the directional direction of the remote sensing y axis
  //��ȡ�ұ�ң��Y�᷽���ģ����  
	PS2_RY=PS2_AnologData(PSS_RY);  

	if(PS2_KEY==4&&PS2_ON_Flag==0) 
		//The start button on the // handle is pressed
		//�ֱ��ϵ�Start����������
		Strat=1; 
	//When the button is pressed, you need to push the right side forward to the formal ps2 control car
	//Start���������º���Ҫ�������ǰ���ˣ��ſ�����ʽPS2����С��
	if(Strat&&(PS2_LY<118)&&PS2_ON_Flag==0)
	{
		PS2_ON_Flag=RC_ON;
		RC_Velocity = 30;
	  RC_Turn_Velocity = Default_Turn_Bias;
	}
	
}

/**************************************************************************
Function: PS2_Control
Input   : none
Output  : none
�������ܣ�PS2�ֱ�����
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void PS2_Control(void)
{
	int LY,RX;									//�ֱ�ADC��ֵ
	int Threshold=20; 							//��ֵ������ҡ��С���ȶ���
	static u8 Key1_Count = 0,Key2_Count = 0;	//���ڿ��ƶ�ȡҡ�˵��ٶ�
	//ת��Ϊ128��-128����ֵ
	LY=-(PS2_LY-128);//���Y�����ǰ������
	RX=-(PS2_RX-128);//�ұ�X�����ת��

	if(LY>-Threshold&&LY<Threshold)	LY=0;
	if(RX>-Threshold&&RX<Threshold)	RX=0;		//����ҡ��С���ȶ���
	
	Move_X = (RC_Velocity/128)*LY;				//�ٶȿ��ƣ����ȱ�ʾ�ٶȴ�С

	if(Move_X>=0)
		Move_Z = -(RC_Turn_Velocity/128)*RX;	//ת����ƣ����ȱ�ʾת���ٶ�
	else
		Move_Z = (RC_Turn_Velocity/128)*RX;
	if (PS2_KEY == PSB_L1) 					 	//������1�����٣������ڶ��ϣ�
	{	
		if((++Key1_Count) == 20)				//���ڰ�����Ӧ�ٶ�
		{
			PS2_KEY = 0;
			Key1_Count = 0;
			if((RC_Velocity += X_Step)>MAX_RC_Velocity)				//���ֵΪ800,���Ϊǰ������ٶ�600mm/s
				RC_Velocity = MAX_RC_Velocity;

				if((RC_Turn_Velocity += Z_Step)>MAX_RC_Turn_Bias)	//ת������ٶ�2000
					RC_Turn_Velocity = MAX_RC_Turn_Bias;
			}
	}
	else if(PS2_KEY == PSB_R1) 					//������1������
	{
		if((++Key2_Count) == 20)
		{
			PS2_KEY = 0;
			Key2_Count = 0;
			if((RC_Velocity -= X_Step)<MINI_RC_Velocity)			//ǰ����С�ٶ�100mm/s
				RC_Velocity = MINI_RC_Velocity;
			
			if((RC_Turn_Velocity -= Z_Step)<MINI_RC_Turn_Velocity)//ת����С�ٶ�200
			   RC_Turn_Velocity = MINI_RC_Turn_Velocity;
		}
	}
	else
		Key2_Count = 0,Key2_Count = 0;			//��ȡ�������������¼���
}


