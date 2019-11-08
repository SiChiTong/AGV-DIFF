#include "bsp_raley.h"
#include "bsp.h"
//#include "sys.h"

void bsp_raley_Init(void)
	
{
/*	
	PC9	  OUT1		���ƻ�ɫ��
	PC8	  OUT2		���ƺ�ɫ��
	PD15  OUT3		������ɫ��
	PD13  OUT5		�̵���	����ǰ��
	PD12  OUT6		�̵���	������ת
	PD9	  OUT7		�̵���	������ת
	PD8   OUT8		�̵���	���Ʊ���
*/

	
	GPIO_InitTypeDef GPIO_InitStructure;
	//IO��ʱ��ʹ��
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��ʱ��GPIOC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//ʹ��ʱ��GPIOD

	//PC9 ��ʼ��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;      //��9Pin
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;  //ͨ�����ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //�ٶȿ���
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//PC8 ��ʼ��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
		
	//PD15 ��ʼ��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	//PD14 ��ʼ��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	//PD13 12��ʼ��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
}



