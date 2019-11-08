/*
*********************************************************************************************************
*
*	ģ������ : ������������ģ��
*	�ļ����� : bsp_key.c
*	��    �� : V1.0
*	˵    �� : ɨ�������������������˲����ƣ����а���FIFO�����Լ�������¼���
*				(1) ��������
*				(2) ��������
*				(3) ������
*				(4) ����ʱ�Զ�����
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*		V1.1    2013-06-29 armfly  ����1����ָ�룬����bsp_Idle() ������ȡϵͳ������ϼ���������
*								   ���� K1 K2 ��ϼ� �� K2 K3 ��ϼ�������ϵͳ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "stm32f4xx_exti.h"
/*
	�ó��������ڰ�����STM32-X3��STM32-V5������

	�����������Ӳ�������޸�GPIO����� IsKeyDown1 - IsKeyDown8 ����

	����û��İ�������С��8��������Խ�����İ���ȫ������Ϊ�͵�1������һ��������Ӱ�������
	#define KEY_COUNT    8	  ����� bsp_key.h �ļ��ж���
*/


/*
**********************************************************************************************************
											�¼���־λ�궨��
**********************************************************************************************************
*/
#define BIT_0	(1 << 0) 	//ͣ��
#define BIT_1	(1 << 1)	//��ת
#define BIT_2	(1 << 2)	//��ת
#define BIT_3	(1 << 3)	//ֱ��



	/* STM32_V5 */
	/*
		������STM32-V5 �������߷��䣺
			������λ		: PE0   (�͵�ƽ��ʾ����)
			������������      : PE1  (�͵�ƽ��ʾ����) 4��̽ͷ��������
			       : PI11  (�͵�ƽ��ʾ����)
			    : PH2   (�͵�ƽ��ʾ����)
			 : PH3   (�͵�ƽ��ʾ����)
			  : PF11  (�͵�ƽ��ʾ����)
			 : PG7   (�͵�ƽ��ʾ����)
			   : PH15  (�͵�ƽ��ʾ����)
	*/


	/* �����ڶ�Ӧ��RCCʱ�� */
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
static KEY_FIFO_T s_tKey;		/* ����FIFO����,�ṹ�� */

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
*	�� �� ��: IsKeyDownX
*	����˵��: �жϰ����Ƿ���
*	��    ��: ��
*	�� �� ֵ: ����ֵ1 ��ʾ���£�0��ʾδ����
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
*	�� �� ��: bsp_InitKey
*	����˵��: ��ʼ������. �ú����� bsp_Init() ���á�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitKey(void)
{
	bsp_InitKeyVar();		/* ��ʼ���������� */
	bsp_InitKeyHard();		/* ��ʼ������Ӳ�� */
	PE0_IN1_INIT();			/* PE0�жϳ�ʼ�� */
	PE1_IN2_INIT();			/* PE1�жϳ�ʼ�� */
	PE2_IN3_INIT();			/* PE2�жϳ�ʼ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PutKey
*	����˵��: ��1����ֵѹ�밴��FIFO��������������ģ��һ��������
*	��    ��:  _KeyCode : ��������
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_GetKey
*	����˵��: �Ӱ���FIFO��������ȡһ����ֵ��
*	��    ��:  ��
*	�� �� ֵ: ��������
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
*	�� �� ��: bsp_GetKey2
*	����˵��: �Ӱ���FIFO��������ȡһ����ֵ�������Ķ�ָ�롣
*	��    ��:  ��
*	�� �� ֵ: ��������
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
*	�� �� ��: bsp_GetKeyState
*	����˵��: ��ȡ������״̬
*	��    ��:  _ucKeyID : ����ID����0��ʼ
*	�� �� ֵ: 1 ��ʾ���£� 0 ��ʾδ����
*********************************************************************************************************
*/
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetKeyParam
*	����˵��: ���ð�������
*	��    �Σ�_ucKeyID : ����ID����0��ʼ
*			_LongTime : �����¼�ʱ��
*			 _RepeatSpeed : �����ٶ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed)
{
	s_tBtn[_ucKeyID].LongTime = _LongTime;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
	s_tBtn[_ucKeyID].RepeatSpeed = _RepeatSpeed;			/* �����������ٶȣ�0��ʾ��֧������ */
	s_tBtn[_ucKeyID].RepeatCount = 0;						/* ���������� */
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_ClearKey
*	����˵��: ��հ���FIFO������
*	��    �Σ���
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKeyHard
*	����˵��: ���ð�����Ӧ��GPIO
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyHard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��1������GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_ALL_KEY, ENABLE);

	/* ��2�����������еİ���GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* �������������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

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
*	�� �� ��: bsp_InitKeyVar
*	����˵��: ��ʼ����������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* �԰���FIFO��дָ������ */
	s_tKey.Read = 0;
	s_tKey.Write = 0;
	s_tKey.Read2 = 0;

	/* ��ÿ�������ṹ���Ա������һ��ȱʡֵ */
	for (i = 0; i < KEY_COUNT; i++)
	{
		s_tBtn[i].LongTime = KEY_LONG_TIME;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
		s_tBtn[i].Count = KEY_FILTER_TIME / 2;		/* ����������Ϊ�˲�ʱ���һ�� */
		s_tBtn[i].State = 0;							/* ����ȱʡ״̬��0Ϊδ���� */
		//s_tBtn[i].KeyCodeDown = 3 * i + 1;				/* �������µļ�ֵ���� */
		//s_tBtn[i].KeyCodeUp   = 3 * i + 2;				/* ��������ļ�ֵ���� */
		//s_tBtn[i].KeyCodeLong = 3 * i + 3;				/* �������������µļ�ֵ���� */
		s_tBtn[i].RepeatSpeed = 0;						/* �����������ٶȣ�0��ʾ��֧������ */
		s_tBtn[i].RepeatCount = 0;						/* ���������� */
	}

	/* �����Ҫ��������ĳ�������Ĳ����������ڴ˵������¸�ֵ */
	/* ���磬����ϣ������1���³���1����Զ��ط���ͬ��ֵ */
	s_tBtn[KID_JOY_U].LongTime = 100;
	s_tBtn[KID_JOY_U].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */

	s_tBtn[KID_JOY_D].LongTime = 100;
	s_tBtn[KID_JOY_D].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */

	s_tBtn[KID_JOY_L].LongTime = 100;
	s_tBtn[KID_JOY_L].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */

	s_tBtn[KID_JOY_R].LongTime = 100;
	s_tBtn[KID_JOY_R].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */

	/* �жϰ������µĺ��� */
	s_tBtn[0].IsKeyDownFunc = IsKeyDown1;
	s_tBtn[1].IsKeyDownFunc = IsKeyDown2;
	s_tBtn[2].IsKeyDownFunc = IsKeyDown3;
	s_tBtn[3].IsKeyDownFunc = IsKeyDown4;
	s_tBtn[4].IsKeyDownFunc = IsKeyDown5;
	s_tBtn[5].IsKeyDownFunc = IsKeyDown6;
	s_tBtn[6].IsKeyDownFunc = IsKeyDown7;
	s_tBtn[7].IsKeyDownFunc = IsKeyDown8;

	/* ��ϼ� */
	s_tBtn[8].IsKeyDownFunc = IsKeyDown9;
	s_tBtn[9].IsKeyDownFunc = IsKeyDown10;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_DetectKey
*	����˵��: ���һ��������������״̬�����뱻�����Եĵ��á�
*	��    ��:  �����ṹ����ָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_DetectKey(uint8_t i)
{
	KEY_T *pBtn;

	/*
		���û�г�ʼ�������������򱨴�
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

				/* ���Ͱ�ť���µ���Ϣ */
				bsp_PutKey((uint8_t)(3 * i + 1));
			}

			if (pBtn->LongTime > 0)
			{
				if (pBtn->LongCount < pBtn->LongTime)
				{
					/* ���Ͱ�ť�������µ���Ϣ */
					if (++pBtn->LongCount == pBtn->LongTime)
					{
						/* ��ֵ���밴��FIFO */
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
							/* ��������ÿ��10ms����1������ */
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

				/* ���Ͱ�ť�������Ϣ */
				bsp_PutKey((uint8_t)(3 * i + 2));
			}
		}

		pBtn->LongCount = 0;
		pBtn->RepeatCount = 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_KeyScan
*	����˵��: ɨ�����а���������������systick�ж������Եĵ���
*	��    ��:  ��
*	�� �� ֵ: ��
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
*	�� �� ��:  PE0_IN1_INIT
*	����˵��:  PE0 ��������ó�ʼ��
*	��    ��:  ��
*	�� �� ֵ:  ��
*********************************************************************************************************
*/
void PE0_IN1_INIT(void)
{
	//�ⲿ�ж�PE0 IN0   ȡ��������ť

	NVIC_InitTypeDef   NVIC_InitStructure;  //�ж�
	EXTI_InitTypeDef   EXTI_InitStructure;  //����exit
	
	//KEY_Init(); //������Ӧ��IO�ڳ�ʼ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	//����IO�����ж��ߵ�ӳ���ϵ
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);//PE0 
	
	
	
	///����EXTI_Line0 ��ʼ�������ж�,���ô���������
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //�����ش��� 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE
	EXTI_Init(&EXTI_InitStructure);//����
	
	
	//�����жϷ��� PE0
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //�ж����ȼ���
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����


}

/*
*********************************************************************************************************
*	�� �� ��:  PE1_IN2_INIT
*	����˵��:  PE0 ��������ó�ʼ��
*	��    ��:  ��
*	�� �� ֵ:  ��
*********************************************************************************************************
*/
void PE1_IN2_INIT(void)
{
	//�ⲿ�ж�PE0 IN0   ȡ��������ť
	
	//NVIC_InitTypeDef   NVIC_InitStructure;  //�ж�
	EXTI_InitTypeDef   EXTI_InitStructure;  //����exit
	
	//KEY_Init(); //������Ӧ��IO�ڳ�ʼ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	//����IO�����ж��ߵ�ӳ���ϵ
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);//PE1
	
	
	
	///����EXTI_Line0 ��ʼ�������ж�,���ô���������
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //�����ش��� 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE
	EXTI_Init(&EXTI_InitStructure);//����
	
//	
//	//�����жϷ��� PE0
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //�ж����ȼ���
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
//    NVIC_Init(&NVIC_InitStructure);//����


}
/*
*********************************************************************************************************
*	�� �� ��:  void PE1_IN2_NVIC(a)
*	����˵��:  ����PE1 IN2 ���ⲿ�жϿ����͹ر�
*	��    ��:  state 
				1   ����
				0   �ر�
*	�� �� ֵ:  ��
*********************************************************************************************************
*/
void PE1_IN2_NVIC(int state)
{
	
	NVIC_InitTypeDef   NVIC_InitStructure;  
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //�ж����ȼ���
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	if(state == 1)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	}
	else if(state == 0)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	}
	NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��:  void PE2_IN3_NVIC(a)
*	����˵��:  ����PE2 IN3 ���ⲿ�жϿ����͹ر�
*	��    ��:  state 
				1   ����
				0   �ر�
*	�� �� ֵ:  ��
*********************************************************************************************************
*/
void PE2_IN3_NVIC(int state)
{
	
	NVIC_InitTypeDef   NVIC_InitStructure;  //�ж�
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //�ж����ȼ���
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	if(state == 1)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	}
	else if(state == 0)
	{
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	}
	NVIC_Init(&NVIC_InitStructure);
}




/*
*********************************************************************************************************
*	�� �� ��:  PE2_IN3_INIT
*	����˵��:  PE0 ��������ó�ʼ��
*	��    ��:  ��
*	�� �� ֵ:  ��
*********************************************************************************************************
*/
void PE2_IN3_INIT(void)
{
	//�ⲿ�ж�PE0 IN0   ȡ��������ť
	
	//NVIC_InitTypeDef   NVIC_InitStructure;  //�ж�
	EXTI_InitTypeDef   EXTI_InitStructure;  //����exit
	
	//KEY_Init(); //������Ӧ��IO�ڳ�ʼ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	//����IO�����ж��ߵ�ӳ���ϵ
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);//PE2 
	
	
	
	///����EXTI_Line0 ��ʼ�������ж�,���ô���������
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;//LINE
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //�����ش��� 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE
	EXTI_Init(&EXTI_InitStructure);//����
	
	
//	//�����жϷ��� PE0
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //�ж����ȼ���
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
//    NVIC_Init(&NVIC_InitStructure);//����


}




/*
*********************************************************************************************************
*	�� �� ��:  PE0_IN1_INIT
*	����˵��:  PE0 �жϷ�����  �ֶ�ֹͣ������ť
*	��    ��:  ��
*	�� �� ֵ:  ��
*********************************************************************************************************
*/
extern  TaskHandle_t xHandle_AGV_Action ;
extern  TaskHandle_t xHandle_Manual ;

void EXTI0_IRQHandler(void) ///�ⲿ�ж�  PE0 
{
	
	bsp_DelayMS(20);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) ==1)
	{
		xTaskNotifyFromISR(xHandle_Manual, /* Ŀ������ */
							BIT_0, /* ����Ŀ�������¼���־λ bit0 */
							eSetBits, /* ��Ŀ��������¼���־λ�� BIT_0 ���л������ �������ֵ���¼���־λ */
							&xHigherPriorityTaskWoken);
		
		/* ��� xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}
	else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) ==0)
	{
		xTaskNotifyFromISR(xHandle_Manual, /* Ŀ������ */
							BIT_1, /* ����Ŀ�������¼���־λ bit1 */
							eSetBits, /* ��Ŀ��������¼���־λ�� BIT_1 ���л������ �������ֵ���¼���־λ */
							&xHigherPriorityTaskWoken);
		
		/* ��� xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}		
	EXTI_ClearITPendingBit(EXTI_Line0); //���LINE0�ϵ��жϱ�־λ 


}

/*
*********************************************************************************************************
*	�� �� ��:  PE1_I2_INIT   ������1��4��̽ͷ
*	����˵��:  PE1 �жϷ�����
*	��    ��:  ��
*	�� �� ֵ:  ��
*********************************************************************************************************
*/
extern  TaskHandle_t xHandle_Forward_Barrier   ;
void EXTI1_IRQHandler(void) ///�������ϰ���  PE1  
{
	//printf("\r\n�ⲿ�ж�\r\n");
	bsp_DelayMS(5);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1) ==1)
	{
		xTaskNotifyFromISR(xHandle_Forward_Barrier  , /* Ŀ������ */
							BIT_0, /* ����Ŀ�������¼���־λ bit0 */
							eSetBits, /* ��Ŀ��������¼���־λ�� BIT_0 ���л������ �������ֵ���¼���־λ */
							&xHigherPriorityTaskWoken);
		
		/* ��� xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}
	else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1) ==0)
	{
		xTaskNotifyFromISR(xHandle_Forward_Barrier  , /* Ŀ������ */
							BIT_1, /* ����Ŀ�������¼���־λ bit1 */
							eSetBits, /* ��Ŀ��������¼���־λ�� BIT_1 ���л������ �������ֵ���¼���־λ */
							&xHigherPriorityTaskWoken);
		
		/* ��� xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}		
	EXTI_ClearITPendingBit(EXTI_Line1); //���LINE1�ϵ��жϱ�־λ 

}

/*
*********************************************************************************************************
*	�� �� ��:  PE2_IN3_INIT   ������5��8��̽ͷ
*	����˵��:  PE2 �жϷ�����
*	��    ��:  ��
*	�� �� ֵ:  ��
*********************************************************************************************************
*/
extern  TaskHandle_t xHandle_BackUp_Barrier;
void EXTI2_IRQHandler(void) ///�������ϰ���  PE2 
{

	bsp_DelayMS(5);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) ==1)
	{
		xTaskNotifyFromISR(xHandle_BackUp_Barrier  , /* Ŀ������ */
							BIT_0, /* ����Ŀ�������¼���־λ bit0 */
							eSetBits, /* ��Ŀ��������¼���־λ�� BIT_0 ���л������ �������ֵ���¼���־λ */
							&xHigherPriorityTaskWoken);
		
		/* ��� xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}
	else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) ==0)
	{
		xTaskNotifyFromISR(xHandle_BackUp_Barrier  , /* Ŀ������ */
							BIT_1, /* ����Ŀ�������¼���־λ bit1 */
							eSetBits, /* ��Ŀ��������¼���־λ�� BIT_1 ���л������ �������ֵ���¼���־λ */
							&xHigherPriorityTaskWoken);
		
		/* ��� xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	}		
	EXTI_ClearITPendingBit(EXTI_Line2); //���LINE2�ϵ��жϱ�־λ 

}







/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
