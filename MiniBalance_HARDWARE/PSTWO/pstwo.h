/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/

#ifndef __PSTWO_H
#define __PSTWO_H

#include "sys.h"


//指示遥控控制的开关
#define RC_ON								1	
#define RC_OFF								0
#define Default_Velocity					300			//默认遥控速度
#define Default_Turn_Bias					800			//默认遥控速度

//前进加减速幅度值，每次遥控加减的步进值
#define X_Step								100
//转弯加减速幅度值
#define Z_Step								300

//遥控控制前后速度最大值
#define MAX_RC_Velocity						800
//遥控控制转向速度最大值
#define	MAX_RC_Turn_Bias					2000
//遥控控制前后速度最小值
#define MINI_RC_Velocity					100
//遥控控制转向速度最小值
#define	MINI_RC_Turn_Velocity			800


/* 定义PS2连接的GPIO端口, 用户只需要修改下面的代码即可改变控制的PS2引脚 */
#define PS2_DI_GPIO_PORT    	GPIOC			              /* GPIO端口 */
#define PS2_DI_GPIO_CLK 	    RCC_APB2Periph_GPIOC			/* GPIO端口时钟 */
#define PS2_DI_GPIO_PIN			GPIO_Pin_13			       		 /* 连接的GPIO */

#define PS2_DO_GPIO_PORT    	GPIOC			              /* GPIO端口 */
#define PS2_DO_GPIO_CLK 	    RCC_APB2Periph_GPIOC			/* GPIO端口时钟 */
#define PS2_DO_GPIO_PIN			GPIO_Pin_14			       		 /* 连接的GPIO */

#define PS2_CS_GPIO_PORT    	GPIOC			              /* GPIO端口 */
#define PS2_CS_GPIO_CLK 	    RCC_APB2Periph_GPIOC			/* GPIO端口时钟 */
#define PS2_CS_GPIO_PIN		    GPIO_Pin_15		       	  	  /* 连接的GPIO */

#define PS2_CLK_GPIO_PORT    	GPIOA			              /* GPIO端口 */
#define PS2_CLK_GPIO_CLK 	    RCC_APB2Periph_GPIOC			/* GPIO端口时钟 */
#define PS2_CLK_GPIO_PIN		GPIO_Pin_2			       		 /* 连接的GPIO */


#define DI   PCin(13)          	//  输入

#define DO_H PCout(14)=1        	//命令位高
#define DO_L PCout(14)=0        	//命令位低

#define CS_H PCout(15)=1       	//CS拉高
#define CS_L PCout(15)=0       	//CS拉低

#define CLK_H PAout(2)=1      	//时钟拉高
#define CLK_L PAout(2)=0      	//时钟拉低




//按键序号
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //右摇杆X轴数据
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

void PS2_Init(void);
u8 PS2_RedLight(void);  	 		//判断是否为红灯模式
void PS2_ReadData(void); 			//读手柄数据
void PS2_Cmd(u8 CMD);		  		//向手柄发送命令
u8 PS2_DataKey(void);		  		//按键值读取
u8 PS2_AnologData(u8 button); 		//得到一个摇杆的模拟量
void PS2_ClearData(void);	  		//清除数据缓冲区
void PS2_Vibration(u8 motor1, u8 motor2);//振动设置motor1  0xFF开，其他关，motor2  0x40~0xFF

void PS2_EnterConfing(void);	 	//进入配置
void PS2_TurnOnAnalogMode(void); 	//发送模拟量
void PS2_VibrationMode(void);    	//振动设置
void PS2_ExitConfing(void);	     	//完成配置
void PS2_SetInit(void);		     	//配置初始化
void PS2_Read(void);
void PS2_Control(void);

#endif

