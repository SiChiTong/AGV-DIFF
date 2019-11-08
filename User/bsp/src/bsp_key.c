/*
*********************************************************************************************************
*
*	模块名称 : 独立按键驱动模块
*	文件名称 : bsp_key.c
*	版    本 : V1.0
*	说    明 : 扫描独立按键，具有软件滤波机制，具有按键FIFO。可以检测如下事件：
*				(1) 按键按下
*				(2) 按键弹起
*				(3) 长按键
*				(4) 长按时自动连发
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*		V1.1    2013-06-29 armfly  增加1个读指针，用于bsp_Idle() 函数读取系统控制组合键（截屏）
*								   增加 K1 K2 组合键 和 K2 K3 组合键，用于系统控制
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "stm32f4xx_exti.h"
/*
	该程序适用于安富莱STM32-X3、STM32-V5开发板

	如果用于其它硬件，请修改GPIO定义和 IsKeyDown1 - IsKeyDown8 函数

	如果用户的按键个数小于8个，你可以将多余的按键全部定义为和第1个按键一样，并不影响程序功能
	#define KEY_COUNT    8	  这个在 bsp_key.h 文件中定义
*/


/*
**********************************************************************************************************
											事件标志位宏定义
**********************************************************************************************************
*/
#define BIT_0	(1 << 0) 	//停车
#define BIT_1	(1 << 1)	//右转
#define BIT_2	(1 << 2)	//左转
#define BIT_3	(1 << 3)	//直行



	/* STM32_V5 */
	/*
		安富莱STM32-V5 按键口线分配：
			启动复位		: PE0   (低电平表示按下)
			超声波传感器      : PE1  (低电平表示按下) 4个探头并联输入
			       : PI11  (低电平表示按下)
			    : PH2   (低电平表示按下)
			 : PH3   (低电平表示按下)
			  : PF11  (低电平表示按下)
			 : PG7   (低电平表示按下)
			   : PH15  (低电平表示按下)
	*/


	/* 按键口对应的RCC时钟 */
	#define RCC_ALL_KEY 	(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOC )

	#define GPIO_PORT_K1    GPIOE
	#define GPIO_PIN_K1	    GPIO_Pin_0

	#define GPIO_PORT_K2    GPIOE
	#define GPIO_PIN_K2	    GPIO_Pin_1

	#define GPIO_PORT_K3    GPIOE
	#define GPIO_PIN_K3	    GPIO_Pin_2

	#define GPIO_PORT_K4    GPIOE
	#define GPIO_PIN_K4	    GPIO_Pin_3

	#define GPIO_PORT_K5    GPIOE
	#define GPIO_PIN_K5	    GPIO_Pin_4

	#define GPIO_PORT_K6    GPIOE
	#define GPIO_PIN_K6	    GPIO_Pin_5

	#define GPIO_PORT_K7    GPIOE
	#define GPIO_PIN_K7	    GPIO_Pin_6

	#define GPIO_PORT_K8    GPIOC
	#define GPIO_PIN_K8	    GPIO_Pin_13

static KEY_T s_tBtn[KEY_COUNT];
static KEY_FIFO_T s_tKey;		/* 按键FIFO变量,结构体 */

static void bsp_InitKeyVar(void);
static void bsp_InitKeyHard(void);
static void bsp_DetectKey(uint8_t i);
void PE0_IN1_INIT(void);
void PE1_IN2_INIT(void);
void PE2_IN3_INIT(void);
void PE1_IN2_NVIC(int state);
void PE2_IN3_NVIC(int state);
/*
*********************************************************************************************************
*	函 数 名: IsKeyDownX
*	功能说明: 判断按键是否按下
*	形    参: 无
*	返 回 值: 返回值1 表示按下，0表示未按下
*********************************************************************************************************
*/

	static uint8_t IsKeyDown1(void) {if ((GPIO_PORT_K1->IDR & GPIO_PIN_K1) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown2(void) {if ((GPIO_PORT_K2->IDR & GPIO_PIN_K2) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown3(void) {if ((GPIO_PORT_K3->IDR & GPIO_PIN_K3) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown4(void) {if ((GPIO_PORT_K4->IDR & GPIO_PIN_K4) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown5(void) {if ((GPIO_PORT_K5->IDR & GPIO_PIN_K5) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown6(void) {if ((GPIO_PORT_K6->IDR & GPIO_PIN_K6) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown7(void) {if ((GPIO_PORT_K7->IDR & GPIO_PIN_K7) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown8(void) {if ((GPIO_PORT_K8->IDR & GPIO_PIN_K8) == 0) return 1;else return 0;}

	static uint8_t IsKeyDown9(void) {if (IsKeyDown1() && IsKeyDown2()) return 1;else return 0;}
	static uint8_t IsKeyDown10(void) {if (IsKeyDown1() && IsKeyDown2()) return 1;else return 0;}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitKey
*	功能说明: 初始化按键. 该函数被 bsp_Init() 调用。
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitKey(void)
{
	bsp_InitKeyVar();		/* 初始化按键变量 */
	bsp_InitKeyHard();		/* 初始化按键硬件 */
	PE0_IN1_INIT();			/* PE0中断初始化 */
	PE1_IN2_INIT();			/* PE1中断初始化 */
	PE2_IN3_INIT();			/* PE2中断初始化 */
}

/*
*********************************************************************************************************
*	函 数 名: bsp_PutKey
*	功能说明: 将1个键值压入按键FIFO缓冲区。可用于模拟一个按键。
*	形    参:  _KeyCode : 按键代码
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_PutKey(uint8_t _KeyCode)
{
	s_tKey.Buf[s_tKey.Write] = _KeyCode;

	if (++s_tKey.Write  >= KEY_FIFO_SIZE)
	{
		s_tKey.Write = 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetKey
*	功能说明: 从按键FIFO缓冲区读取一个键值。
*	形    参:  无
*	返 回 值: 按键代码
*********************************************************************************************************
*/
uint8_t bsp_GetKey(void)
{
	uint8_t ret;

	if (s_tKey.Read == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read];

		if (++s_tKey.Read >= KEY_FIFO_SIZE)
		{
			s_tKey.Read = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetKey2
*	功能说明: 从按键FIFO缓冲区读取一个键值。独立的读指针。
*	形    参:  无
*	返 回 值: 按键代码
*********************************************************************************************************
*/
uint8_t bsp_GetKey2(void)
{
	uint8_t ret;

	if (s_tKey.Read2 == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read2];

		if (++s_tKey.Read2 >= KEY_FIFO_SIZE)
		{
			s_tKey.Read2 = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetKeyState
*	功能说明: 读取按键的状态
*	形    参:  _ucKeyID : 按键ID，从0开始
*	返 回 值: 1 表示按下， 0 表示未按下
*********************************************************************************************************
*/
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetKeyParam
*	功能说明: 设置按键参数
*	形    参：_ucKeyID : 按键ID，从0开始
*			_LongTime : 长按事件时间
*			 _RepeatSpeed : 连发速度
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed)
{
	s_tBtn[_ucKeyID].LongTime = _LongTime;			/* 长按时间 0 表示不检测长按键事件 */
	s_tBtn[_ucKeyID].RepeatSpeed = _RepeatSpeed;			/* 按键连发的速度，0表示不支持连发 */
	s_tBtn[_ucKeyID].RepeatCount = 0;						/* 连发计数器 */
}


/*
*********************************************************************************************************
*	函 数 名: bsp_ClearKey
*	功能说明: 清空按键FIFO缓冲区
*	形    参：无
*	返 回 值: 按键代码
*********************************************************************************************************
*/
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitKeyHard
*	功能说明: 配置按键对应的GPIO
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitKeyHard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 第1步：打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_ALL_KEY, ENABLE);

	/* 第2步：配置所有的按键GPIO为浮动输入模式(实际上CPU复位后就是输入状态) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* 设为输入口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 无需上下拉电阻 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K1;
	GPIO_Init(GPIO_PORT_K1, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K2;
	GPIO_Init(GPIO_PORT_K2, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K3;
	GPIO_Init(GPIO_PORT_K3, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K4;
	GPIO_Init(GPIO_PORT_K4, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K5;
	GPIO_Init(GPIO_PORT_K5, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K6;
	GPIO_Init(GPIO_PORT_K6, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K7;
	GPIO_Init(GPIO_PORT_K7, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K8;
	GPIO_Init(GPIO_PORT_K8, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitKeyVar
*	功能说明: 初始化按键变量
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* 对按键FIFO读写指针清零 */
	s_tKey.Read = 0;
	s_tKey.Write = 0;
	s_tKey.Read2 = 0;

	/* 给每个按键结构体成员变量赋一组缺省值 */
	for (i = 0; i < KEY_COUNT; i++)
	{
		s_tBtn[i].LongTime = KEY_LONG_TIME;			/* 长按时间 0 表示不检测长按键事件 */
		s_tBtn[i].Count = KEY_FILTER_TIME / 2;		/* 计数器设置为滤波时间的一半 */
		s_tBtn[i].State = 0;							/* 按键缺省状态，0为未按下 */
		//s_tBtn[i].KeyCodeDown = 3 * i + 1;				/* 按键按下的键值代码 */
		//s_tBtn[i].KeyCodeUp   = 3 * i + 2;				/* 按键弹起的键值代码 */
		//s_tBtn[i].KeyCodeLong = 3 * i + 3;				/* 按键被持续按下的键值代码 */
		s_tBtn[i].RepeatSpeed = 0;						/* 按键连发的速度，0表示不支持连发 */
		s_tBtn[i].RepeatCount = 0;						/* 连发计数器 */
	}

	/* 如果需要单独更改某个按键的参数，可以在此单独重新赋值 */
	/* 比如，我们希望按键1按下超过1秒后，自动重发相同键值 */
	s_tBtn[KID_JOY_U].LongTime = 100;
	s_tBtn[KID_JOY_U].RepeatSpeed = 5;	/* 每隔50ms自动发送键值 */

	s_tBtn[KID_JOY_D].LongTime = 100;
	s_tBtn[KID_JOY_D].RepeatSpeed = 5;	/* 每隔50ms自动发送键值 */

	s_tBtn[KID_JOY_L].LongTime = 100;
	s_tBtn[KID_JOY_L].RepeatSpeed = 5;	/* 每隔50ms自动发送键值 */

	s_tBtn[KID_JOY_R].LongTime = 100;
	s_tBtn[KID_JOY_R].RepeatSpeed = 5;	/* 每隔50ms自动发送键值 */

	/* 判断按键按下的函数 */
	s_tBtn[0].IsKeyDownFunc = IsKeyDown1;
	s_tBtn[1].IsKeyDownFunc = IsKeyDown2;
	s_tBtn[2].IsKeyDownFunc = IsKeyDown3;
	s_tBtn[3].IsKeyDownFunc = IsKeyDown4;
	s_tBtn[4].IsKeyDownFunc = IsKeyDown5;
	s_tBtn[5].IsKeyDownFunc = IsKeyDown6;
	s_tBtn[6].IsKeyDownFunc = IsKeyDown7;
	s_tBtn[7].IsKeyDownFunc = IsKeyDown8;

	/* 组合键 */
	s_tBtn[8].IsKeyDownFunc = IsKeyDown9;
	s_tBtn[9].IsKeyDownFunc = IsKeyDown10;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_DetectKey
*	功能说明: 检测一个按键。非阻塞状态，必须被周期性的调用。
*	形    参:  按键结构变量指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_DetectKey(uint8_t i)
{
	KEY_T *pBtn;

	/*
		如果没有初始化按键函数，则报错
		if (s_tBtn[i].IsKeyDownFunc == 0)
		{
			printf("Fault : DetectButton(), s_tBtn[i].IsKeyDownFunc undefine");
		}
	*/

	pBtn = &s_tBtn[i];
	if (pBtn->IsKeyDownFunc())
	{
		if (pBtn->Count < KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count < 2 * KEY_FILTER_TIME)
		{
			pBtn->Count++;
		}
		else
		{
			if (pBtn->State == 0)
			{
				pBtn->State = 1;

				/* 发送按钮按下的消息 */
				bsp_PutKey((uint8_t)(3 * i + 1));
			}

			if (pBtn->LongTime > 0)
			{
				if (pBtn->LongCount < pBtn->LongTime)
				{
					/* 发送按钮持续按下的消息 */
					if (++pBtn->LongCount == pBtn->LongTime)
					{
						/* 键值放入按键FIFO */
						bsp_PutKey((uint8_t)(3 * i + 3));
					}
				}
				else
				{
					if (pBtn->RepeatSpeed > 0)
					{
						if (++pBtn->RepeatCount >= pBtn->RepeatSpeed)
						{
							pBtn->RepeatCount = 0;
							/* 常按键后，每隔10ms发送1个按键 */
							bsp_PutKey((uint8_t)(3 * i + 1));
						}
					}
				}
			}
		}
	}
	else
	{
		if(pBtn->Count > KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count != 0)
		{
			pBtn->Count--;
		}
		else
		{
			if (pBtn->State == 1)
			{
				pBtn->State = 0;

				/* 发送按钮弹起的消息 */
				bsp_PutKey((uint8_t)(3 * i + 2));
			}
		}

		pBtn->LongCount = 0;
		pBtn->RepeatCount = 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_KeyScan
*	功能说明: 扫描所有按键。非阻塞，被systick中断周期性的调用
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_KeyScan(void)
{
	uint8_t i;

	for (i = 0; i < KEY_COUNT; i++)
	{
		bsp_DetectKey(i);
	}
}



/*
*********************************************************************************************************
*	函 数 名:  PE0_IN1_INIT
*	功能说明:  PE0 输入口配置初始化
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
void PE0_IN1_INIT(void)
{
	//外部中断PE0 IN0   取料启动按钮

	NVIC_InitTypeDef   NVIC_InitStructure;  //中断
	EXTI_InitTypeDef   EXTI_InitStructure;  //配置exit
	
	//KEY_Init(); //按键对应的IO口初始化
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	//配置IO口与中断线的映射关系
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);//PE0 
	
	
	
	///配置EXTI_Line0 初始化线上中断,设置触发条件等
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升沿触发 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE
	EXTI_Init(&EXTI_InitStructure);//配置
	
	
	//配置中断分组 PE0
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //中断优先级组
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置


}

/*
*********************************************************************************************************
*	函 数 名:  PE1_IN2_INIT
*	功能说明:  PE0 输入口配置初始化
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
void PE1_IN2_INIT(void)
{
	//外部中断PE0 IN0   取料启动按钮
	
	//NVIC_InitTypeDef   NVIC_InitStructure;  //中断
	EXTI_InitTypeDef   EXTI_InitStructure;  //配置exit
	
	//KEY_Init(); //按键对应的IO口初始化
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	//配置IO口与中断线的映射关系
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);//PE1
	
	
	
	///配置EXTI_Line0 初始化线上中断,设置触发条件等
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升沿触发 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE
	EXTI_Init(&EXTI_InitStructure);//配置
	
//	
//	//配置中断分组 PE0
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //中断优先级组
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
//    NVIC_Init(&NVIC_InitStructure);//配置


}
/*
*********************************************************************************************************
*	函 数 名:  void PE1_IN2_NVIC(a)
*	功能说明:  控制PE1 IN2 的外部中断开启和关闭
*	形    参:  state 
				1   开启
				0   关闭
*	返 回 值:  无
*********************************************************************************************************
*/
void PE1_IN2_NVIC(int state)
{
	
	NVIC_InitTypeDef   NVIC_InitStructure;  
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //中断优先级组
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	if(state == 1)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	}
	else if(state == 0)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	}
	NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名:  void PE2_IN3_NVIC(a)
*	功能说明:  控制PE2 IN3 的外部中断开启和关闭
*	形    参:  state 
				1   开启
				0   关闭
*	返 回 值:  无
*********************************************************************************************************
*/
void PE2_IN3_NVIC(int state)
{
	
	NVIC_InitTypeDef   NVIC_InitStructure;  //中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //中断优先级组
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	if(state == 1)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	}
	else if(state == 0)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	}
	NVIC_Init(&NVIC_InitStructure);
}




/*
*********************************************************************************************************
*	函 数 名:  PE2_IN3_INIT
*	功能说明:  PE0 输入口配置初始化
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
void PE2_IN3_INIT(void)
{
	//外部中断PE0 IN0   取料启动按钮
	
	//NVIC_InitTypeDef   NVIC_InitStructure;  //中断
	EXTI_InitTypeDef   EXTI_InitStructure;  //配置exit
	
	//KEY_Init(); //按键对应的IO口初始化
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	//配置IO口与中断线的映射关系
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);//PE2 
	
	
	
	///配置EXTI_Line0 初始化线上中断,设置触发条件等
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;//LINE
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升沿触发 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE
	EXTI_Init(&EXTI_InitStructure);//配置
	
	
//	//配置中断分组 PE0
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //中断优先级组
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
//    NVIC_Init(&NVIC_InitStructure);//配置


}




/*
*********************************************************************************************************
*	函 数 名:  PE0_IN1_INIT
*	功能说明:  PE0 中断服务函数  手动停止启动按钮
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
extern  TaskHandle_t xHandle_AGV_Action ;
extern  TaskHandle_t xHandle_Manual ;

void EXTI0_IRQHandler(void) ///外部中断  PE0 
{
	
	bsp_DelayMS(20);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) ==1)
	{
		xTaskNotifyFromISR(xHandle_Manual, /* 目标任务 */
							BIT_0, /* 设置目标任务事件标志位 bit0 */
							eSetBits, /* 将目标任务的事件标志位与 BIT_0 进行或操作， 将结果赋值给事件标志位 */
							&xHigherPriorityTaskWoken);
		
		/* 如果 xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}
	else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) ==0)
	{
		xTaskNotifyFromISR(xHandle_Manual, /* 目标任务 */
							BIT_1, /* 设置目标任务事件标志位 bit1 */
							eSetBits, /* 将目标任务的事件标志位与 BIT_1 进行或操作， 将结果赋值给事件标志位 */
							&xHigherPriorityTaskWoken);
		
		/* 如果 xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}		
	EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位 


}

/*
*********************************************************************************************************
*	函 数 名:  PE1_I2_INIT   超声波1到4号探头
*	功能说明:  PE1 中断服务函数
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
extern  TaskHandle_t xHandle_Forward_Barrier   ;
void EXTI1_IRQHandler(void) ///超声波障碍物  PE1  
{
	//printf("\r\n外部中断\r\n");
	bsp_DelayMS(5);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1) ==1)
	{
		xTaskNotifyFromISR(xHandle_Forward_Barrier  , /* 目标任务 */
							BIT_0, /* 设置目标任务事件标志位 bit0 */
							eSetBits, /* 将目标任务的事件标志位与 BIT_0 进行或操作， 将结果赋值给事件标志位 */
							&xHigherPriorityTaskWoken);
		
		/* 如果 xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}
	else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1) ==0)
	{
		xTaskNotifyFromISR(xHandle_Forward_Barrier  , /* 目标任务 */
							BIT_1, /* 设置目标任务事件标志位 bit1 */
							eSetBits, /* 将目标任务的事件标志位与 BIT_1 进行或操作， 将结果赋值给事件标志位 */
							&xHigherPriorityTaskWoken);
		
		/* 如果 xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}		
	EXTI_ClearITPendingBit(EXTI_Line1); //清除LINE1上的中断标志位 

}

/*
*********************************************************************************************************
*	函 数 名:  PE2_IN3_INIT   超声波5到8号探头
*	功能说明:  PE2 中断服务函数
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
extern  TaskHandle_t xHandle_BackUp_Barrier;
void EXTI2_IRQHandler(void) ///超声波障碍物  PE2 
{

	bsp_DelayMS(5);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) ==1)
	{
		xTaskNotifyFromISR(xHandle_BackUp_Barrier  , /* 目标任务 */
							BIT_0, /* 设置目标任务事件标志位 bit0 */
							eSetBits, /* 将目标任务的事件标志位与 BIT_0 进行或操作， 将结果赋值给事件标志位 */
							&xHigherPriorityTaskWoken);
		
		/* 如果 xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}
	else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) ==0)
	{
		xTaskNotifyFromISR(xHandle_BackUp_Barrier  , /* 目标任务 */
							BIT_1, /* 设置目标任务事件标志位 bit1 */
							eSetBits, /* 将目标任务的事件标志位与 BIT_1 进行或操作， 将结果赋值给事件标志位 */
							&xHigherPriorityTaskWoken);
		
		/* 如果 xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}		
	EXTI_ClearITPendingBit(EXTI_Line2); //清除LINE2上的中断标志位 

}







/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
