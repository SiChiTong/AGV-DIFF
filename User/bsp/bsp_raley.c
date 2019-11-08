#include "bsp_raley.h"
#include "bsp.h"
//#include "sys.h"

void bsp_raley_Init(void)
	
{
/*	
	PC9	  OUT1		控制黄色灯
	PC8	  OUT2		控制红色灯
	PD15  OUT3		控制绿色灯
	PD13  OUT5		继电器	控制前进
	PD12  OUT6		继电器	控制左转
	PD9	  OUT7		继电器	控制右转
	PD8   OUT8		继电器	控制报警
*/

	
	GPIO_InitTypeDef GPIO_InitStructure;
	//IO口时钟使能
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能时钟GPIOC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//使能时钟GPIOD

	//PC9 初始化
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;      //第9Pin
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;  //通用输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //速度控制
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//PC8 初始化
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
		
	//PD15 初始化
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	//PD14 初始化
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	//PD13 12初始化
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
}



