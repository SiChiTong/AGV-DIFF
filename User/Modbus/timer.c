#include "timer.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "bsp.h"
//#include "stm32f4xx_tim.h"
static void (*s_TIM_CallBack)(void);
static uint32_t g_iRunTime = 0;

void MODBUS_Tim2_Init(void)
{
	//��ʱ��2���ڼ���,
	//APB1����ʱ��4��Ƶ Ϊ 42MHZ
	//TIM3��ʱ��ʱ��ΪAPB1����ʱ�ӵ�2��, Ϊ84MHZ
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint32_t usPeriod;
	uint16_t usPrescaler;
	uint32_t uiTIMxCLK;

  	/* ʹ��TIMʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	uiTIMxCLK = SystemCoreClock / 2;    //168/2 = 84

	usPrescaler = uiTIMxCLK / 1000000 ;	/* �����Ƶϵ��    ��Ƶ������ 1us   ����Ҫ-1*/ 

	usPeriod = 0xFFFF;	/* F407֧��32λ */
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = usPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM7�ж�,��������ж�
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIMx enable counter */
	TIM_Cmd(TIM2, DISABLE);

	/* ����TIM��ʱ�ж� (Update) */
	{
		NVIC_InitTypeDef NVIC_InitStructure;	/* �жϽṹ���� misc.h �ж��� */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  //�ж����ȼ���
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;

		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	/* �ȴ������ȼ��� */
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}

//���������ж��Ƿ񾭹�3.5���ֽڵ�ʱ��
//�ڴ��ڽ����жϺ����б����ã�ÿ����һ������һ�Σ����¿�ʼ��ʱ
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
	
    cnt_now = TIM_GetCounter(TIM2);    	/* ��ȡ��ǰ�ļ�����ֵ */
    cnt_tar = cnt_now + _uiTimeOut;			/* ���㲶��ļ�����ֵ */
    
    s_TIM_CallBack = (void (*)(void))_pCallBack;

    TIM_SetCompare3(TIM2, cnt_tar);      	/* ���ò���Ƚϼ�����CC3 */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);	/* ʹ��CC3�ж� */
	
}

//���û�������ݽ��ܣ���ʱ�����жϺ���������־λ��1
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
        TIM_ITConfig(TIM2, TIM_IT_CC3, DISABLE);	/* ����CC3�ж� */
		
        s_TIM_CallBack();	//����־λ��1
		
		TIM_Cmd(TIM2, DISABLE);
    }
}

//��ʱ��3���ڼ���,1ms�����ж�
	//APB1����ʱ��4��Ƶ Ϊ 42MHZ
	//TIM3��ʱ��ʱ��ΪAPB1����ʱ�ӵ�2��, Ϊ84MHZ
void MODBUS_Tim3_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  	/* ʹ��TIMʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	

	
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* TIMx enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* ����TIM��ʱ�ж� (Update) */
	{
		NVIC_InitTypeDef NVIC_InitStructure;	/* �жϽṹ���� misc.h �ж��� */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  //�ж����ȼ���
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;

		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	/* �ȴ������ȼ��� */
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

//���������ж��Ƿ�ʱ
//��ȡ��ǰ����������ֵ
//void WFI_SET(void);		//ִ��WFIָ��
//void INTX_DISABLE(void);//�ر������ж�
//void INTX_ENABLE(void);	//���������ж�
uint32_t bsp_GetRunTime(void)
{
	int32_t runtime;

	//WFI_SET();
	__set_PRIMASK(1);
	//__ASM volatile ("cpsid i" : : : "memory");/* ���ж� */

	runtime = g_iRunTime;	/* ���������Systick�ж��б���д�������Ҫ���жϽ��б��� */

	//__ASM volatile ("cpsie i" : : : "memory");/* ���ж� */
	//WFI_SET();
	__set_PRIMASK(0);
	return runtime;
}

//��ȡ���ʱ��
uint32_t bsp_CheckRunTime(int32_t _LastTime)
{
	int32_t now_time;
	int32_t time_diff;

	//__ASM volatile ("cpsid i" : : : "memory");/* ���ж� */
	//WFI_SET();
	__set_PRIMASK(1);
	now_time = g_iRunTime;	/* ���������Systick�ж��б���д�������Ҫ���жϽ��б��� */

	//__set_PRIMASK(0);   		/* ���ж� */
	//ENABLE_INT();  	
	//WFI_SET();
	__set_PRIMASK(0);
	//__ASM volatile ("cpsie i" : : : "memory");/* ���ж� */
	
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

