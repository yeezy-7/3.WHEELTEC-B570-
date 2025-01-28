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
Version: 5.7
Update��2021-04-29

All rights reserved
***********************************************/
#include "control.h"
u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
/**************************************************************************
Function: Control function
Input   : none
Output  : none
�������ܣ����еĿ��ƴ��붼��������
         5ms�ⲿ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��	
��ڲ�������
����  ֵ����				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//��ѹ������ر���
	static u8 Flag_Target;																//���ƺ�����ر������ṩ10ms��׼
	int Encoder_Left,Encoder_Right;             					//���ұ��������������
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//ƽ�⻷PWM�������ٶȻ�PWM������ת��PWM��
	if(INT==0)		
	{   
		EXTI->PR=1<<12;                           					//����жϱ�־λ   
		Encoder_Left=-Read_Encoder(3);            					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
		Encoder_Right=-Read_Encoder(4);           					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
		Get_Angle(Way_Angle);                     					//������̬��5msһ�Σ����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
		Flag_Target=!Flag_Target;																										
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//����������ת�ٶȣ�mm/s��
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0,LD_Successful_Receive_flag=0;  		//���������ṩ50ms�ľ�׼��ʱ��ʾ������Ҫ50ms�߾�����ʱ
		}
		if(Flag_Target==1)                        					//10ms����һ��
		{
			Voltage_Temp=Get_battery_volt();		    					//��ȡ��ص�ѹ		
			Voltage_Count++;                       						//ƽ��ֵ������
			Voltage_All+=Voltage_Temp;              					//��β����ۻ�
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ		
			return 0;	                                               
		}                                         					//10ms����һ��
        if(Mode==Ultrasonic_Avoid_Mode||Mode==Ultrasonic_Follow_Mode)		
	       Read_Distane();			                           //��ȡ��������������ֵ
		if(Flag_follow==0&&Flag_avoid==0)	Led_Flash(100);   //LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
		if(Flag_follow==1||Flag_avoid==1)	Led_Flash(0);     //LED����;����������/����ģʽ	
		Key();                                    			 //ɨ�谴��״̬ ����˫�����Ըı�С������״̬
		PS2_Control();                                       //�ֱ����Ʋ��ܺ�Ѳ��ͬ�¿���
		Select_Zhongzhi();                                   //��е��ֵ��ѡ��
		Lidar_Avoid();                                       //�״����ģʽ
		Lidar_Follow();                                      //�״����ģʽ
		Lidar_Straight();                                    //�״���ֱ��ģʽ
		CCD_Mode();                                          //CCDѲ��
		ELE_Mode();                                          //���Ѳ��
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //ƽ��PID���� Gyro_Balanceƽ����ٶȼ��ԣ�ǰ��Ϊ��������Ϊ��
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //�ٶȻ�PID����	��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
		if(Mode ==CCD_Line_Patrol_Mode)                     //CCDѭ���µ�ת�򻷿��� 
			Turn_Pwm=CCD_turn(CCD_Zhongzhi,Gyro_Turn);
		else if(Mode==ELE_Line_Patrol_Mode)                 //ELEѭ���µ�ת�򻷿���
			Turn_Pwm=ELE_turn(Gyro_Turn);
		else
		   Turn_Pwm=Turn(Gyro_Turn);						//ת��PID����     
		
		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //�������ֵ������PWM
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //�������ֵ������PWM
															//PWMֵ����ʹС��ǰ��������ʹС������
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM�޷�
		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//����Ƿ�С��������
			Flag_Stop=1;	                           					//���������͹رյ��
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//����Ƿ�С��������
			Flag_Stop=0;	                           					//��������¾��������
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//����������쳣
			Set_Pwm(Motor_Left,Motor_Right);         					//��ֵ��PWM�Ĵ���  
	 }       	
	 return 0;	  
} 

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle��Gyro��angular velocity
Output  : balance��Vertical control PWM
�������ܣ�ֱ��PD����		
��ڲ�����Angle:�Ƕȣ�Gyro�����ٶ�
����  ֵ��balance��ֱ������PWM
**************************************************************************/	
int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//���ƽ��ĽǶ���ֵ �ͻ�е���
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left��Left wheel encoder reading��encoder_right��Right wheel encoder reading
Output  : Speed control PWM
�������ܣ��ٶȿ���PWM		
��ڲ�����encoder_left�����ֱ�����������encoder_right�����ֱ���������
����  ֵ���ٶȿ���PWM
**************************************************************************/
//�޸�ǰ�������ٶȣ����޸�Target_Velocity�����磬�ĳ�60�ͱȽ�����
int Velocity(int encoder_left,int encoder_right)
{  
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral,Target_Velocity;
	  //================ң��ǰ�����˲���====================// 
		if(Flag_follow==1||Flag_avoid==1) Target_Velocity = 30; //����������/����ģʽ,�����ٶ�
		else 											        Target_Velocity = 50;
		if(Flag_front==1)    	Movement=Target_Velocity/Flag_velocity;	  //�յ�ǰ���ź�
		else if(Flag_back==1)	Movement=-Target_Velocity/Flag_velocity;  //�յ������ź�
	    else  Movement=Move_X;
	
   //=============���������ܣ�����/���ϣ�==================// 
	  if(Mode==Ultrasonic_Follow_Mode&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //����
			 Movement=Target_Velocity/Flag_velocity;
		if(Mode==Ultrasonic_Follow_Mode&&Distance<200&&Flag_Left!=1&&Flag_Right!=1) 
			 Movement=-Target_Velocity/Flag_velocity;
		if(Mode==Ultrasonic_Avoid_Mode&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //����������
			 Movement=-Target_Velocity/Flag_velocity;
		
   //================�ٶ�PI������=====================//	
		Encoder_Least =0-(encoder_left+encoder_right);                    //��ȡ�����ٶ�ƫ��=Ŀ���ٶȣ��˴�Ϊ�㣩-�����ٶȣ����ұ�����֮�ͣ� 
		Encoder_bias *= 0.84;		                                          //һ�׵�ͨ�˲���       
		Encoder_bias += Encoder_Least*0.16;	                              //һ�׵�ͨ�˲����������ٶȱ仯 
		Encoder_Integral +=Encoder_bias;                                  //���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //����ң�������ݣ�����ǰ������
		if(Encoder_Integral>110000)  	Encoder_Integral=110000;             //�����޷�
		if(Encoder_Integral<-110000)	  Encoder_Integral=-110000;            //�����޷�	
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //�ٶȿ���	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//����رպ��������
	  return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
�������ܣ�ת����� 
��ڲ�����Z��������
����  ֵ��ת�����PWM
��    �ߣ���Ȥ�Ƽ�����ݸ�����޹�˾ 
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//�޸�ת���ٶȣ����޸�Turn_Amplitude����
	//===================ң��������ת����=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
	 else Turn_Target=0;
	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;        
	 else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  //===================ת��PD������=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100+Move_Z;//���Z�������ǽ���PD����
	 return turn;								 				 //ת��PWM��תΪ������תΪ��
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left��Left wheel PWM��motor_right��Right wheel PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	    AIN1=1,			AIN2=0; //ǰ�� 
	else           			  AIN1=0,			AIN2=1; //����
	PWMA=myabs(motor_left);	
  if(motor_right>0)			BIN1=1,			BIN2=0;	//ǰ��
	else 	        			  BIN1=0,			BIN2=1; //����
	PWMB=myabs(motor_right);
}
/**************************************************************************
Function: PWM limiting range
Input   : IN��Input  max��Maximum value  min��Minimum value
Output  : Output
�������ܣ�����PWM��ֵ 
��ڲ�����IN���������  max���޷����ֵ  min���޷���Сֵ
����  ֵ���޷����ֵ
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(50); 
	if(tmp==1)
	{ 
		Flag_Stop=!Flag_Stop;
	}		//��������С������ͣ
	tmp2=Long_Press();                   
  if(tmp2==1) Flag_Show=!Flag_Show;	//�������ƽ�����λ��ģʽ��С������ʾֹͣ

}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle��Car inclination��voltage��Voltage
Output  : 1��abnormal��0��normal
�������ܣ��쳣�رյ��		
��ڲ�����angle��С����ǣ�voltage����ѹ
����  ֵ��1���쳣  0������
**************************************************************************/	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000)//��ص�ѹ����10V�رյ��
	{	                                                 //��Ǵ���40�ȹرյ��
		temp=1;                                          //Flag_Stop��1�����������ƹرյ��
		AIN1=0;                                            
		AIN2=0;
		BIN1=0;
		BIN2=0;
	}
	else
		temp=0;
	return temp;			
}
	
/**************************************************************************
Function: Get angle
Input   : way��The algorithm of getting angle 1��DMP  2��kalman  3��Complementary filtering
Output  : none
�������ܣ���ȡ�Ƕ�	
��ڲ�����way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	if(way==1)                           //DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{	
		Read_DMP();                      	 //��ȡ���ٶȡ����ٶȡ����
		Angle_Balance=Pitch;             	 //����ƽ�����,ǰ��Ϊ��������Ϊ��
		Gyro_Balance=gyro[0];              //����ƽ����ٶ�,ǰ��Ϊ��������Ϊ��
		Gyro_Turn=gyro[2];                 //����ת����ٶ�
		Acceleration_Z=accel[2];           //����Z����ٶȼ�
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		if(Gyro_X>32768)  Gyro_X-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //��������ת��
		if(Accel_X>32768) Accel_X-=65536;                //��������ת��
		if(Accel_Y>32768) Accel_Y-=65536;                //��������ת��
		if(Accel_Z>32768) Accel_Z-=65536;                //��������ת��
		Gyro_Balance=-Gyro_X;                            //����ƽ����ٶ�
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //������ǣ�ת����λΪ��	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //������ǣ�ת����λΪ��
		accel_x=Accel_X/16384;                           //MPU6050��ʼ��Ϊ���ٶȼ�Ϊ��2g���õ���ԭʼ����Ϊ16λ���ݣ��������λΪ����λ��
		accel_y=Accel_Y/16384;                           //���Զ�ȡ��������λ��32768����Ӧ��2g������16384�������ݷֱ��ʣ�ԭʼ���ݳ���16384�õ�����Ϊm/S^2
		accel_z=Accel_Z/16384;
		gyro_x=Gyro_X/16.4;                              //����������ת�������̡�2000��/s��Ӧ������16.4���õ�ԭʼ����Ϊ����32768����Ӧ��2000��/s
		gyro_y=Gyro_Y/16.4;                              //����32768/2000 = 16.4��Ҳ�ɲ鿴�ֲ�õ�������
		if(Way_Angle==2)		  	
		{
			 Pitch= -Kalman_Filter_x(Accel_Angle_x,gyro_x);//�������˲�����λΪ��
			 Roll = -Kalman_Filter_y(Accel_Angle_y,gyro_y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//�����˲�
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //����ƽ�����
		Gyro_Turn=Gyro_Z;                                 //����ת����ٶ�
		Acceleration_Z=Accel_Z;                           //����Z����ٶȼ�
	}

}
/**************************************************************************
Function: Absolute value function
Input   : a��Number to be converted
Output  : unsigned int
�������ܣ�����ֵ����
��ڲ�����a����Ҫ�������ֵ����
����  ֵ���޷�������
**************************************************************************/	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration��Z-axis acceleration��Angle��The angle of balance��encoder_left��Left encoder count��encoder_right��Right encoder count
Output  : 1��picked up  0��No action
�������ܣ����С���Ƿ�����
��ڲ�����Acceleration��z����ٶȣ�Angle��ƽ��ĽǶȣ�encoder_left���������������encoder_right���ұ���������
����  ֵ��1:С��������  0��С��δ������
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //��һ��
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //����1��С���ӽ���ֹ
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //����ڶ���
	 {
			if(++count1>200)       count1=0,flag=0;                       //��ʱ���ٵȴ�2000ms�����ص�һ��
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //����2��С������0�ȸ���������
			flag=2; 
	 } 
	 if(flag==2)                                                       //������
	 {
		  if(++count2>100)       count2=0,flag=0;                        //��ʱ���ٵȴ�1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //����3��С������̥��Ϊ�������ﵽ����ת��   
      {
				flag=0;                                                                                     
				return 1;                                                    //��⵽С��������
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance��Left encoder count��Right encoder count
Output  : 1��put down  0��No action
�������ܣ����С���Ƿ񱻷���
��ڲ�����ƽ��Ƕȣ���������������ұ���������
����  ֵ��1��С������   0��С��δ����
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                     //��ֹ���      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //����1��С������0�ȸ�����
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //��ʱ���ٵȴ� 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //����2��С������̥��δ�ϵ��ʱ����Ϊת��  
      {
				flag=0;
				flag=0;
				return 1;                         //��⵽С��������
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
�������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//���ת��  ת��=������������5msÿ�Σ�*��ȡƵ��/��Ƶ��/���ٱ�/����������
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
}
/**************************************************************************
Function: Select car running mode
Input   : encoder_left��Left wheel encoder reading��encoder_right��Right wheel encoder reading
Output  : none
�������ܣ�ѡ��С������ģʽ
��ڲ�����encoder_left�������������  encoder_right���ұ���������
����  ֵ����
**************************************************************************/
u8 Choose(void)
{
	static int count;
	u8 tmp;
    oled_show_once();
	count += myabs(Read_Encoder(4));
	if(count>6&&count<180)	//��ͨģʽ
	{
		Mode = 0;
	}
	if(count>180&&count<360)	//����������ģʽ 
	{	
		Mode = 1;
	}
	if(count>360&&count<540)	//����������ģʽ
	{		
		Mode = 2;
	}
	if(count>540&&count<720)   //�״����
	{
		Mode = 3;
	}
	if(count>720&&count<900)   //�״����
	{
		Mode = 4;
	}
	if(count>900&&count<1080)   //�״���ֱ��
	{
		Mode = 5;
	}
	if(count>1080&&count<1260)   //CCDѲ��
	{
		Mode = 6;
	}
	if(count>1260&&count<1440)   //���Ѳ��
	{
		Mode = 7;
	}
	if(count>1440)
		Mode=0,count=0;
	tmp=click_N_Double(50);
	if(tmp==1) 
	{
		OLED_Clear();
		return 0;
	}
	else return 1;
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
�������ܣ��״�ǰ������ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Avoid(void)
{
	u8 i;
	u8 avoid_Num=0;//��Ҫ���ϵĵ�
	float Angle_Sum=0;//ȷ���ϰ�������һ����ı���
	u8 too_close = 0;//�ж��ϰ����Ƿ�̫���ı���
	if(Mode==Lidar_Avoid_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			if((Dataprocess[i].angle<avoid_Angle1)||(Dataprocess[i].angle>avoid_Angle2))//С��ǰ������100�ȷ�Χ
			{		
				if((Dataprocess[i].distance>0)&&(Dataprocess[i].distance<avoid_Distance))//����С��300mm��Ҫ����
				{
					Distance=Dataprocess[i].distance;
					avoid_Num++;
					if(Dataprocess[i].angle>310) Angle_Sum += (Dataprocess[i].angle-360);//��310~360ת��Ϊ-50��0
					else if(Dataprocess[i].angle<50) Angle_Sum+= Dataprocess[i].angle;
					if(Dataprocess[i].distance<150)			too_close++;//����̫������Ҫ����
				}
			}
		}
		if(avoid_Num<8)
		{
		  Move_X=avoid_speed;                                           //��С��һ��200mm/s���ٶȣ���Ҫ����800
			Move_Z=0;
		}
		else if(avoid_Num>8)
		{
			 Move_X=0;
	    	if(too_close>10) Move_X=-avoid_speed,Move_Z=0;              //����̫��������һ��
				else
				{
					if(Angle_Sum>0)      
					{
						Move_Z=-turn_speed;//�ϰ��￿�ң���ת
					}
					else   Move_Z=turn_speed; //�ϰ��￿����ת
				}					
		}
 }
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
�������ܣ��״����ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Follow(void)
{
	u8 i;
	u8 follow_num=0;                //�жϸ���ĵ�
	u16 mini_distance = 65535;      //Ҫ����ľ��룬������С�����ľ���
	static float angle =0;                 //�����ĽǶ�
	static float last_angle = 0;           //��������һ���Ƕ�
	u8 data_count = 0;
	if(Mode==Lidar_Follow_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			 if((0<Dataprocess[i].distance) && (Dataprocess[i].distance<Follow_distance))//��0~1500mm��ѡ������ĵ�������
			 {
				 follow_num++;
				 if(Dataprocess[i].distance<mini_distance)                  //�жϳ���С����ĵ�
				 {
					 mini_distance = Dataprocess[i].distance;
					 angle = Dataprocess[i].angle;
					 Distance = mini_distance;                                     //��oled����ʾҪ�����ľ���
				 }
			 }
	  }
	if(angle>180)
		  angle -= 360;				//0--360��ת����0--180��-180--0��˳ʱ�룩
	if(angle-last_angle>10 ||angle-last_angle<-10)	//��һ����������������10�ȵ���Ҫ���ж�
	{
		if(++data_count == 60)		//����60�βɼ�����ֵ(300ms��)���ϴεıȴ���10�ȣ���ʱ������Ϊ����Чֵ
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else							//����С��10�ȵĿ���ֱ����Ϊ����Чֵ
	{
			data_count = 0;	
			last_angle = angle;
	}
	if(follow_num>5) 	
	{
		Move_X=Lidar_follow_PID(mini_distance,300);//����ľ���pidʱֱ���������ٶȻ�������Ҫ��Сһ��(Move�ķ�Χ��0~800)
		Move_Z=Follow_Turn_PID(angle,0);//ת��PIDֱ��������ת��
	}
	else
	{
		Move_X = 0;
		Move_Z = 0;
	}
	if(Move_X>60)    Move_X=60;
 }
}
/**************************************************************************
Function: Lidar_Straight
Input   : none
Output  : none
�������ܣ��״���ֱ��ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Straight(void) 
{
	static u16 target_distance=0;
	u8 i;
	u16 current_distance=target_distance;
	static u16 Limit_distance=0;   //�״�����̽�����
	if(Mode==Lidar_Straight_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		 Move_X=Initial_speed;//��С��һ����ʼ�ٶ�
		 for(i=0;i<225;i++)
	  {
		  if((Dataprocess[i].angle>71)&&(Dataprocess[i].angle<74))//ȡ�״��70��75�ȷ�Χ�ĵ����Ƚϵ�
		 {
			 if(determine<Limit_time) //��ģʽת����Straightģʽ3���ȷ��������Ҫ�ľ���
			 {
				 target_distance=Dataprocess[i].distance;
				 Limit_distance=target_distance+200;//��Ŀ������200mm,��Ҫ������������ʧ����С������ת��
				 determine++;
				 if(determine==(Limit_time-1)) determine=Limit_time;
			 }
			 if(Dataprocess[i].distance<(Limit_distance))//����һ���״��̽�����
			 {
				 current_distance=Dataprocess[i].distance;//ȷ������
			   Distance=Dataprocess[i].distance;
			 }
		 }
	 }
	 Move_Z=Distance_Adjust_PID(current_distance,target_distance);//�״����pid
	}
}

/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ
     for(i=5;i<123;i++)   //���߸�ȥ��5����
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //��Сֵ
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //���������������ȡ����ֵ
	 for(i = 5;i<118; i++)   //Ѱ�����������
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//Ѱ���ұ�������
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//��������λ��
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫��
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ��
}

/**************************************************************************
Function: Detect_Barrier
Input   : none
Output  : 1or0(Barrier_Detected or No_Barrier)
�������ܣ����Ѳ��ģʽ�״����ϰ���
��ڲ���: ��
����  ֵ��1��0(��⵽�ϰ�������ϰ���)
**************************************************************************/	 	
//����ϰ���
u8 Detect_Barrier(void)
{
	u8 i;
	u8 point_count = 0;
	if(Lidar_Detect == Lidar_Detect_ON)
	{
		for(i=0;i<225;i++)	//����Ƿ����ϰ���
		{
			if(Dataprocess[i].angle>340 || Dataprocess[i].angle <20) //��С������ǰ��40�㷶Χ
			{
				if(0<Dataprocess[i].distance&&Dataprocess[i].distance<Detect_distance)//700mm���Ƿ����ϰ���
				  point_count++,Distance=Dataprocess[i].distance;
			}
		}
		if(point_count > 3)//���ϰ���
			return Barrier_Detected;
		else
			return No_Barrier;
	}
	else
		return No_Barrier;
}

/**************************************************************************
Function: CCD_Mode
Input   : none
Output  : none
�������ܣ�CCDѲ��ģʽ����
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void CCD_Mode(void)
{
	static u8 Count_CCD = 0;								//����CCD����Ƶ��
	if(Mode == CCD_Line_Patrol_Mode && Flag_Left !=1 &&Flag_Right !=1)
	{
		Move_X = tracking_speed;			//CCDѲ���ٶ�
		if(++Count_CCD == 4)								//���ڿ���Ƶ�ʣ�4*5 = 20ms����һ��
		{
			RD_TSL(); 
            Find_CCD_Zhongzhi();			
            Count_CCD = 0;			
		}
//		else if(Count_CCD>4)  Count_CCD = 0;
	    
		
	}
}

/**************************************************************************
�������ܣ�CCDģʽת�����  Ѳ��
��ڲ�����CCD��ȡ������ Z��������
����  ֵ��ת�����PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int CCD_turn(u8 CCD,float gyro)//ת�����
{
	  float Turn;     
      float Bias,kp=30,Kd=0.12;	  
	  Bias=CCD-64;
	  Turn=Bias*kp+gyro*Kd;
	  return Turn;
}

/**************************************************************************
Function: ELE_Mode
Input   : none
Output  : none
�������ܣ����Ѳ��ģʽ����
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void ELE_Mode(void)
{
	if(Mode == ELE_Line_Patrol_Mode && Flag_Left!=1 &&Flag_Right!=1)
	{
		int Sum = 0;
		Sensor_Left = Get_Adc(2);
		Sensor_Middle = Get_Adc(1);
		Sensor_Right = Get_Adc(9);
		Sum = Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;			
		Sensor = Sum/(Sensor_Left+Sensor_Middle+Sensor_Right);
//		if(Detect_Barrier() == No_Barrier)		//��⵽���ϰ���
				Move_X=tracking_speed;       //��С��һ�����300mm/s���ٶ�
//		else									//���ϰ���
//		{
//			Move_X = 0;
//		}	
	}
	
}

/**************************************************************************
�������ܣ�ELEģʽת�����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int ELE_turn(float gyro)//ת�����
{
	float Turn;     
	float Bias,kp=60,Kd=0.2;	  
	Bias=Sensor-100;
	Turn=Bias*kp+gyro*Kd;
	  return Turn;
}


/**************************************************************************
Function: Select_Zhongzhi
Input   : none
Output  : none
�������ܣ�С����е��ֵ��ѡ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Select_Zhongzhi(void)                   //��е��ֵѡ�񣬱��ⰲװ�ϵ��Ѳ�ߡ�CCDѲ��װ��ʱС����ǰ�������
{
	if(Mode == ELE_Line_Patrol_Mode)
		Middle_angle = -9;
	else if(Mode == CCD_Line_Patrol_Mode)
		Middle_angle = -4;
	else   Middle_angle = 0;
}
