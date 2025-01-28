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

#include "Lidar.h"
#include <string.h>
#include "sys.h"



float Follow_KP =10,Follow_KD =1,Follow_KI = 0.001;	
float Velocity_KP = 1.1,Velocity_KI = 1;			//����ʽPI���������ڵ���ٶȿ��ƣ���������������

PointDataProcessDef PointDataProcess[225] ;//����225������
LiDARFrameTypeDef Pack_Data;
PointDataProcessDef Dataprocess[225];      //����С�����ϡ����桢��ֱ�ߡ�ELE�״���ϵ��״�����

/**************************************************************************
Function: LIDAR_USART_Init
Input   : none
Output  : none
�������ܣ��״ﴮ�ڳ�ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
//����5

void LIDAR_USART_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// �򿪴��������ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	

	//����USARTΪ�ж�Դ 
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//�����ȼ� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//ʹ���ж� 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);


	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate =230400;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(USART2, &USART_InitStructure);

	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	
	//USART_ITConfig(UART5, USART_IT_TXE, ENABLE);

	// ʹ�ܴ���
	USART_Cmd(USART2, ENABLE);	    

}

/**************************************************************************
Function: data_process
Input   : none
Output  : none
�������ܣ����ݴ�����
��ڲ�������
����  ֵ����
**************************************************************************/
//���һ֡���պ���д���
void data_process(void) //���ݴ���
{
	static u8 data_cnt = 0;
	u8 i,m,n;
	u32 distance_sum[8]={0};//2����ľ���͵�����
	LD_Successful_Receive_flag=1;
	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//����16����Ŀ�ʼ�Ƕ�
	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//����16����Ľ����Ƕ�
	float area_angle[8]={0};//һ֡���ݵ�8��ƽ���Ƕ�
	if((start_angle>350)&&(end_angle<13))//��Ϊһ֡������13�ȣ�����350��10��η�Χ��ӣ����angle�������180��Χ
		end_angle +=360;
	for(m=0;m<8;m++)
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/8*m;
		if(area_angle[m]>360)  area_angle[m] -=360;
	}
	for(i=0;i<16;i++)
	{
		switch(i)
		{
			case 0:case 1:
				distance_sum[0] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//0~1��ľ����
			  break;
			case 2:case 3:
				distance_sum[1] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//2~3��ľ����
			  break;
			case 4:case 5:
				distance_sum[2] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//4~5��ľ����
			  break;
			case 6:case 7:
				distance_sum[3] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//6~7��ľ����
			  break;
			case 8:case 9:
				distance_sum[4] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//8~9��ľ����
			  break;
			case 10:case 11:
				distance_sum[5] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//10~11��ľ����
			  break;
			case 12:case 13:
				distance_sum[6] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//12~13��ľ����
			  break;
			case 14:case 15:
				distance_sum[7] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//14~15��ľ����
			  break;
			default:break;
								
		}
		  
	}
	for(n=0;n<8;n++)
	{
		PointDataProcess[data_cnt+n].angle = area_angle[n];
	  PointDataProcess[data_cnt+n].distance = distance_sum[n]/2;
	}
	data_cnt +=8;
	if(data_cnt>=225)
	{
		for(i=0;i<225;i++)
		{
			Dataprocess[i].angle=PointDataProcess[i].angle;
			Dataprocess[i].distance=PointDataProcess[i].distance;
		}
		data_cnt = 0;
	}
		
}

/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void USART2_IRQHandler(void)
{	

	static u8 state = 0;//״̬λ	
	static u8 crc_sum = 0;//У���
	static u8 cnt = 0;//����һ֡16����ļ���
	u8 temp_data;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //���յ�����
	{	
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		temp_data=USART_ReceiveData(USART2);	
		switch(state)
		{
			case 0:
				if(temp_data == HEADER_0)//ͷ�̶�
				{
					Pack_Data.header_0= temp_data;
					state++;
					//У��
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 1:
				if(temp_data == HEADER_1)//ͷ�̶�
				{
					Pack_Data.header_1 = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 2:
				if(temp_data == Length_)//�ֳ��̶�
				{
					Pack_Data.ver_len = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 3:
				Pack_Data.speed_h = temp_data;//�ٶȸ߰�λ
				state++;
				crc_sum += temp_data;			
				break;
			case 4:
				Pack_Data.speed_l = temp_data;//�ٶȵͰ�λ
				state++;
				crc_sum += temp_data;
				break;
			case 5:
				Pack_Data.start_angle_h = temp_data;//��ʼ�Ƕȸ߰�λ
				state++;
				crc_sum += temp_data;
				break;
			case 6:
				Pack_Data.start_angle_l = temp_data;//��ʼ�ǶȵͰ�λ
				state++;
				crc_sum += temp_data;
				break;
			
			case 7:case 10:case 13:case 16:
			case 19:case 22:case 25:case 28:
			case 31:case 34:case 37:case 40:
			case 43:case 46:case 49:case 52:
				Pack_Data.point[cnt].distance_h = temp_data;//16����ľ������ݣ����ֽ�
				state++;
				crc_sum += temp_data;
				break;
			
			case 8:case 11:case 14:case 17:
			case 20:case 23:case 26:case 29:
			case 32:case 35:case 38:case 41:
			case 44:case 47:case 50:case 53:
				Pack_Data.point[cnt].distance_l = temp_data;//16����ľ������ݣ����ֽ�
				state++;
				crc_sum += temp_data;
				break;
			
			case 9:case 12:case 15:case 18:
			case 21:case 24:case 27:case 30:
			case 33:case 36:case 39:case 42:
			case 45:case 48:case 51:case 54:
				Pack_Data.point[cnt].Strong = temp_data;//16�����ǿ������
				state++;
				crc_sum += temp_data;
				cnt++;
				break;
			
			case 55:
				Pack_Data.end_angle_h = temp_data;//�����Ƕȵĸ߰�λ
				state++;
				crc_sum += temp_data;			
				break;
			case 56:
				Pack_Data.end_angle_l = temp_data;//�����ǶȵĵͰ�λ
				state++;
				crc_sum += temp_data;
				break;
			case 57:
				Pack_Data.crc = temp_data;//У��
				state = 0;
				cnt = 0;
				if(crc_sum == Pack_Data.crc)
				{
					data_process();//���ݴ���У����ȷ����ˢ�´洢������
				}
				else 
				{
					memset(&Pack_Data,0,sizeof(Pack_Data));//����
				}
				crc_sum = 0;//У�������
				break;
			default: break;
		}
	}		

}


/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ��״����pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
//�״�������pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	OutPut=-2.5*Bias-100*(Bias-Last_Bias);//λ��ʽPID������  //
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	return OutPut;                                          	
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ��״�������pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	

float Lidar_follow_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	OutPut=-0.15*Bias-0.1*(Bias-Last_Bias);//λ��ʽPID������  //
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
�������ܣ��״�ת��pid
��ڲ���: ��ǰ�ǶȺ�Ŀ��Ƕ�
����  ֵ�����ת���ٶ�
**************************************************************************/	 	
//�״�ת��pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)		                                 				 //���ƫ��Ļ���
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //����ƫ��
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=-Follow_KP*Bias-Follow_KI*Integral_bias-Follow_KD*(Bias-Last_Bias);	//λ��ʽPID������
	Last_Bias=Bias;                                       					 		//������һ��ƫ��
	if(Turn_Off(Angle_Balance,Voltage)== 1)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                           					 	//���
	
}







