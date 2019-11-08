#include "timer.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "bsp.h"
//#include "stm32f4xx_tim.h"
static void (*s_TIM_CallBack)(void);
static uint32_t g_iRunTime = 0;

void MODBUS_Tim2_Init(void)
{
	//定时器2用于计数,
	//APB1总线时钟4分频 为 42MHZ
	//TIM3定时器时钟为APB1总线时钟的2倍, 为84MHZ
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint32_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;

  	/* 使能TIM时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	uiTIMxCLK = SystemCoreClock / 2;    //168/2 = 84

	usPrescaler = uiTIMxCLK / 1000000 ;	/* 计算分频系数    分频到周期 1us   正常要-1*/ 

	usPeriod = 0xFFFF;	/* F407支持32位 */
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = usPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM7中断,允许更新中断
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIMx enable counter */
	TIM_Cmd(TIM2, DISABLE);

	/* 配置TIM定时中断 (Update) */
	{
		NVIC_InitTypeDef NVIC_InitStructure;	/* 中断结构体在 misc.h 中定义 */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  //中断优先级组
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;

		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	/* 比串口优先级低 */
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}

//以下用来判断是否经过3.5个字节的时间
//在串口接受中断函数中被调用，每接受一个调用一次，重新开始计时
void bsp_StartHardTimer(uint32_t _uiTimeOut, void * _pCallBack)
{
	uint32_t cnt_now;
    uint32_t cnt_tar;

    if (_uiTimeOut < 5)
    {
        ;
    }
    else
    {
        _uiTimeOut -= 5;
    }
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);
	
    cnt_now = TIM_GetCounter(TIM2);    	/* 读取当前的计数器值 */
    cnt_tar = cnt_now + _uiTimeOut;			/* 计算捕获的计数器值 */
    
    s_TIM_CallBack = (void (*)(void))_pCallBack;

    TIM_SetCompare3(TIM2, cnt_tar);      	/* 设置捕获比较计数器CC3 */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);	/* 使能CC3中断 */
	
}

//如果没有新数据接受，超时进入中断函数，将标志位置1
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
        TIM_ITConfig(TIM2, TIM_IT_CC3, DISABLE);	/* 禁能CC3中断 */
		
        s_TIM_CallBack();	//将标志位置1
		
		TIM_Cmd(TIM2, DISABLE);
    }
}

//定时器3用于计数,1ms进入中断
	//APB1总线时钟4分频 为 42MHZ
	//TIM3定时器时钟为APB1总线时钟的2倍, 为84MHZ
void MODBUS_Tim3_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  	/* 使能TIM时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	

	
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* TIMx enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* 配置TIM定时中断 (Update) */
	{
		NVIC_InitTypeDef NVIC_InitStructure;	/* 中断结构体在 misc.h 中定义 */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  //中断优先级组
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;

		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	/* 比串口优先级低 */
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
		g_iRunTime++;
		if(g_iRunTime > 0x7FFFFFFF) g_iRunTime = 0;
    }    
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

//以下用来判断是否超时
//获取当前计数器计数值
//void WFI_SET(void);		//执行WFI指令
//void INTX_DISABLE(void);//关闭所有中断
//void INTX_ENABLE(void);	//开启所有中断
uint32_t bsp_GetRunTime(void)
{
	int32_t runtime;

	//WFI_SET();
	__set_PRIMASK(1);
	//__ASM volatile ("cpsid i" : : : "memory");/* 关中断 */

	runtime = g_iRunTime;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

	//__ASM volatile ("cpsie i" : : : "memory");/* 开中断 */
	//WFI_SET();
	__set_PRIMASK(0);
	return runtime;
}

//获取间隔时间
uint32_t bsp_CheckRunTime(int32_t _LastTime)
{
	int32_t now_time;
	int32_t time_diff;

	//__ASM volatile ("cpsid i" : : : "memory");/* 关中断 */
	//WFI_SET();
	__set_PRIMASK(1);
	now_time = g_iRunTime;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

	//__set_PRIMASK(0);   		/* 开中断 */
	//ENABLE_INT();  	
	//WFI_SET();
	__set_PRIMASK(0);
	//__ASM volatile ("cpsie i" : : : "memory");/* 开中断 */
	
	if (now_time >= _LastTime)
	{
		time_diff = now_time - _LastTime;
	}
	else
	{
		time_diff = 0x7FFFFFFF - _LastTime + now_time;
	}

	return time_diff;
}

