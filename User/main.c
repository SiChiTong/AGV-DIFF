/*
*********************************************************************************************************
*
关于直线修正速度，设置阶梯
假设偏移1个检测点，直线修正速度为 A
则偏移2个检测点，直线修正速度为 2A



*
*	
*
*********************************************************************************************************
*/
#include "includes.h"


#include "modbus_host.h"
#include "RFID.h"

#define  Turn_Time     60// 单位s


#define  Auto_Wait     0
#define  Auto_Stop     1
#define  Auto_Straight 2
#define  Auto_Left	   3
#define  Auto_Right    4
#define  Auto_Crossing 5


/*
分岔路路径选择标志位
*/
#define Straight_Flag 1
#define Right_Flag    2
#define Left_Flag     3
#define Back_Flag     4

//#define  Safe_Distance 0x64 //单位是厘米  100cm
#define  Safe_Distance 0x1E //单位是厘米  30cm

#define OPEN 1
#define CLOSE 0

int count_Left =2;
int count_Right =2;

uint16_t Read_Speed_1=0;
uint16_t Read_Speed_2=0;

int Read_Speed_1_H=0;
int Read_Speed_1_L=0;

int Read_Speed_2_H=0;
int Read_Speed_2_L=0;


/*
**********************************************************************************************************
											AGV设备标识
**********************************************************************************************************
*/
#define  ZZBM	0xA3 			//制造部门 制造3
#define  Plant  0x2B 			//车间     2号机加车间
#define  Num    0x01 			//号码     1号AGV
#define  Network_Head_LEN   2	//包头长度
#define  Network_End_LEN    1	//包尾长度

#define  Network_State_LEN   1	//装填长度
#define  Network_Place_LEN   1	//地点长度
#define  Network_Reserved_LEN 8
#define  Network_AGV_BUF_LEN  17
 
 
#define Manual_Mode   		0xF1    //手动模式
#define Auto_Mode	  		0xF2	//自动模式
 
 
 
 
#define Standby   		0xC1    //待机
#define Parking	  		0xC2	//站点停车
#define Moving    		0xC3	//行驶中
#define Derail_ERR 		0xC4	//脱轨报警
#define Barrier_ERR  	0xC5	//障碍物报警
#define Manual_Stop  	0xC6	//障碍物报警


#define Forward     1 //前进方向
#define Back_Up     2 //后退方向

#define Straight_Len     10 //直行段数组长度
#define Left_Len         10 //左转段数组长度
#define Right_Len        10 //右转段数组长度
#define Back_Len		 10 //后退段数组长度


uint8_t Work_Mode ;


uint8_t Network_Head_BUF[Network_Head_LEN] = {0x17,0x18};


uint8_t Network_AGVID_BUF[3]={0,0,0};


uint8_t Network_End_BUF[Network_End_LEN] = {0xFF};

uint8_t Network_Reserved_BUF[Network_Reserved_LEN] = {0x00};



uint8_t A[Network_State_LEN] ={0x33};
uint8_t B[Network_Place_LEN] ={0x44};

uint8_t Place;
uint8_t State  = Standby ;//默认处于待机状态

uint8_t Network_AGV_BUF[Network_AGV_BUF_LEN]= {0x00};


uint8_t State_01 =0; //01号探头状态机标志位
uint8_t State_16 =0; //16号探头状态机标志位

uint8_t Straight_Sets[Straight_Len]   = {0xC1,0xC2,0xC3,0xC4,0xC5};
uint8_t Left_Turn_Sets[Left_Len]      = {0xB1,0xB2,0xB3,0xB4,0xB5};
uint8_t Right_Turn_Sets[Right_Len]    = {0xD1,0xD2,0xD3,0xD4,0xD5};
uint8_t Back_Turn_Sets[Back_Len]      = {0xA1,0xA2,0xA3,0xA4,0xA5};




/*
**********************************************************************************************************
											事件标志位宏定义
**********************************************************************************************************
*/
#define BIT_0	(1 << 0) 	//停车
#define BIT_1	(1 << 1)	//右转
#define BIT_2	(1 << 2)	//左转
#define BIT_3	(1 << 3)	//直行
#define BIT_4	(1 << 4)	//直行,当检测到磁轨道的探头数大于7




/*
**********************************************************************************************************
											任务函数声明
**********************************************************************************************************
*/

static void AppTaskCreate (void); //用于创建任务

static void vTask_U2_Navigation(void *pvParameters); //串口2 磁导航任务
static void vTask_U3_RFID(void *pvParameters);		 //串口3 RFID读卡器任务
static void vTask_U5_Navigation(void *pvParameters); //串口5 磁导航任务
static void vTask_U6_HMI(void *pvParameters);        //串口6 HMI人机界面任务
//static void vTask_Working(void *pvParameters);     //工作任务
static void vTaskCheck(void *pvParameters);        	 //检测外设任务


static void vTask_Manual(void *pvParameters);        	//手动按钮
static void vTask_Forward_Barrier(void *pvParameters);        	//障碍物中断任务 前方

static void vTask_BackUp_Barrier(void *pvParameters);        	//障碍物中断任务  后方


///AGV动作任务vTask_AGV_Action
static void vTask_AGV_Action(void *pvParameters);       

static void vTask_AGV_To_Network(void *pvParameters);       //发送AGV状态任务


static void vTask_Left_90Angel_Check(void *pvParameters); //AGV左转90度检测任务
static void vTask_Right_90Angel_Check(void *pvParameters); //AGV右转90度检测任务







/*
**********************************************************************************************************
											任务句柄声明
**********************************************************************************************************
*/
static TaskHandle_t xHandle_U2_Navigation = NULL;
static TaskHandle_t xHandle_U3_RFID = NULL;
static TaskHandle_t xHandle_U5_Navigation = NULL;
static TaskHandle_t xHandle_U6_HMI = NULL;
static TaskHandle_t xHandlevTaskCheck = NULL;

TaskHandle_t xHandle_AGV_Action  = NULL;
TaskHandle_t xHandle_Manual = NULL;
static TaskHandle_t xHandle_AGV_To_Network  = NULL;
TaskHandle_t xHandle_Forward_Barrier  = NULL;
TaskHandle_t xHandle_BackUp_Barrier  = NULL;

static TaskHandle_t xHandle_Left_90Angel_Check  = NULL;
static TaskHandle_t xHandle_Right_90Angel_Check  = NULL;

	
////二值信号量句柄
//SemaphoreHandle_t xSemaphore;	//二值信号量句柄
//QueueHandle_t USART1_Queue;   		
//QueueHandle_t Message_Queue;	

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
**********************************************************************************************************
											被调函数声明
**********************************************************************************************************
*/

uint8_t Verify_RFID(uint8_t *data,uint8_t* RFID); //IC卡检测函数
void RFID_Check(void); 		//U3  RFID检测函数
void U2_Navigation_Check(void);//U2  磁导航检测函数
void U5_Navigation_Check(void);//U5  超声波检测函数

void Auto_Work(void);		//工作函数
void Go_Straight(void);     //直行函数
void Stop(void);			//停车函数
void Alarm(void);			//报警函数
void Turn_Left(uint8_t *data);		//左转函数
void Turn_Right(uint8_t *data);		//右转函数
void U2_State(void);		//U2  磁导航数据格式检测状态机函数
void U2_State_Check(void);	//U2  磁导航数据格式检测状态机函数  专门用于初始化
void U3_State(void);		//U3  RFID数据格式检测状态机函数
void U5_State(void);		//U5  姿态传感器数据格式检测波状态机函数
void U6_State(void);		//U6  HMI数据格式检测状态机函数
void U2_Deal(uint8_t *data);//U2  磁导航数据处理函数
void U3_Deal(uint8_t *data);//U3  RFID数据数据处理函数
void U5_Deal(uint8_t *data);//U5  超声数据数据处理函数
void U5_State_Check(void);	//U2  磁导航数据格式检测状态机函数  专门用于初始化
void U6_Deal(uint8_t *data);//U6  HMI数据数据处理函数
void HMI_END(void);			//HMI结束符指令
void Discern_IC_Card(void);	//IC卡识别函数
void Servo_Init(void);		//伺服电机初始化函数
void RFID_TO_HMI(uint8_t place); //RFID识别目的地后反馈给HMI 函数
void Network_Init(void);       	//网络初始化函数

void AGV_To_Network(uint8_t *_Work_Mode,uint8_t *_Reserved);   //通过网络发送AGV状态函数



void Network_Head_Send(void);    //发送网络通讯包头
void Network_End_Send(void);     //发送网络通讯包尾
void Network_AGVID_Send(void);   //发送AGV ID


void Network_AGVState_Send(uint8_t *_State1,uint8_t *_Place1); //发送AGV状态

void Network_Reserved_Send(uint8_t *Reserved); //发送AGV信息（预留）
void Network_AGVWork_Send(uint8_t *_Work_Mode);//发送AGV工作模式

void AGV_Have_Barrier(void); //检测到障碍物
void AGV_None_Barrier(void); //障碍物移除

void AGV_StopBManual(void);  //手动停止工作
void AGV_RunBManual(void);	 //手动恢复工作


void Task_Resume(void); //恢复任务

void Time_Out(void);//检查转弯是否超时

int Path_Choice(uint8_t Order_Place);//分岔路选择
void Fork_Left(void); //分岔路左转
void Fork_Right(void);//分岔路右转
void Fork_Straight(void);//分岔路直行
void Fork_Back(void);//分岔路后退

void Read_Speed(void);//读取电机速度
void Send_Speed(void);//发布电机速度给触摸屏
void go_straight(void);//直走
void go_back(void);//后退
/*
**********************************************************************************************************
											转弯函数声明
**********************************************************************************************************
*/
void Left_90Angel(void); //左转90度
void Right_90Angel(void);//右转90度

void U2_Left_90Angel_Check(void);  //通过U2串口的磁导航检测是否完成左转90度
void U2_Right_90Angel_Check(void); //通过U2串口的磁导航检测是否完成右转90度

void U5_Left_90Angel_Check(void);  //通过U5串口的磁导航检测是否完成左转90度
void U5_Right_90Angel_Check(void); //通过U5串口的磁导航检测是否完成右转90度

/*
**********************************************************************************************************
											指示灯函数声明
**********************************************************************************************************
*/

void Running_Green_Light(void);  //正常工作时 绿色灯亮
void Waiting_Yellow_Light(void); //站点停车等待或者左右转弯90度时，黄色灯亮
void Alarm_Red_Light(void);      //故障报警时候，红色灯亮

/*
**********************************************************************************************************
											全局变量声明
**********************************************************************************************************
*/
uint8_t Guidance_State = 0; //磁导航状态（停车，左转，右转，直行）
uint8_t PLACE_Match    = 0; //磁导航状态（停车，左转，右转，直行）目的地符合标志位

////串口2	磁导航
uint8_t U2_read;
uint8_t U2_ucStatus = 0;  /* 状态机标志 */
uint8_t U2_ucCount=0;
uint8_t U2_buf[20];
///////////////

////串口3	RFID读卡器
uint8_t U3_read;
uint8_t U3_ucStatus = 0;  /* 状态机标志 */
uint8_t U3_ucCount=0;
uint8_t U3_buf[20];
///////////////


////串口5	磁导航
uint8_t U5_read;
uint8_t U5_ucStatus = 0;  /* 状态机标志 */
uint8_t U5_ucCount=0;
uint8_t U5_buf[20];
///////////////

////串口6	HMI
uint8_t U6_read;
uint8_t U6_ucStatus = 0;  /* 状态机标志 */
uint8_t U6_ucCount=0;
uint8_t U6_buf[30];
///////////////

uint8_t* RFID_IN;


uint8_t	Manual_Place = 0; //上位机手动指定地点模式
uint8_t	Auto_Place   = 0; //上位机自动指定地点模式

uint8_t Place_Stop = 0;//目的地标志位
uint8_t	Send_First = 0;//HMI第一笔数据标志位
uint8_t Stop_Flag  = 0; //已经停车标志位 防止重复发速度指令
uint8_t Derail = 0; //出轨标志位
uint8_t PLACE_Gone = 0;//记录已经达到目的地数量

uint8_t	Left  = 0;//已经左转标志位 防止重复发速度指令
uint8_t	Right = 0;//已经右转标志位 防止重复发速度指令
uint8_t	Stop_change = 0;

uint8_t UART2_Done=0; 	//串口2完成一次数据处理
uint8_t UART3_Done =0;	//串口3完成一次数据处理
uint8_t UART5_Done =0;	//串口5完成一次数据处理
uint8_t UART6_Done =0;	//串口6完成一次数据处理


uint8_t U2_ucStatus_Check = 0; 
uint8_t U5_ucStatus_Check = 0; 
uint8_t U2_Navigation_Checked =0;
uint8_t U5_Navigation_Checked =0;
uint8_t U2_Navigation_Correct = 0;//串口2 磁导航传感器    检测成功标志位
uint8_t U5_Navigation_Correct = 0;//串口5 磁导航传感器    检测成功标志位

uint8_t Attitude_Correct = 0; //串口5 姿态传感器  检测成功标志位

uint8_t HMI_Command[30];//保存HMI指令的数组

uint8_t Barrier = 66; //障碍物标志位

uint8_t new_data1;
uint8_t new_data2;

/*





速度定义，差速，电机速度

速度差 = 快的轮子转速 - 慢的轮子转速





*/
uint16_t Speed_Straight = 320; //直走时驱动电机转速  快的轮子速度是400

uint16_t Speed_Difference  = 12; //速度差 慢的轮子速度是400-96=304  16比较稳定

uint16_t Speed_90Angle = 90;//左右转转90度的速度   快的轮子速度是90

uint16_t Speed_90Angle_Difference = 30;//左右转转90度的速度    慢的轮子速度是90-30=60

uint16_t Speed_Straight_GO = 90;//测试直行时候的速度

uint16_t HMI_Straight_Speed   = 0;  //直行速度
uint16_t HMI_Correction_Speed = 0;	//直行修正速度
uint16_t HMI_Turn_90_Speed    = 0;  //左右转90度速度



uint16_t Motor1_Speed = 0;  //左上 1号电机速度
uint16_t Motor2_Speed = 0;	//右下 2号电机速度

uint16_t Direction = Forward; //默认是方向是前进方向，会影响电机正反转

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{
	/* 
	  在启动调度前，为了防止初始化STM32外设时有中断服务程序执行，这里禁止全局中断(除了NMI和HardFault)。
	  这样做的好处是：
	  1. 防止执行的中断服务程序中有FreeRTOS的API函数。
	  2. 保证系统正常启动，不受别的中断影响。
	  3. 关于是否关闭全局中断，大家根据自己的实际情况设置即可。
	  在移植文件port.c中的函数prvStartFirstTask中会重新开启全局中断。通过指令cpsie i开启，__set_PRIMASK(1)
	  和cpsie i是等效的。
     */
	__set_PRIMASK(1);  
	
	/* 硬件初始化 */
	bsp_Init(); 
	
	PE1_IN2_NVIC(OPEN);
	PE2_IN3_NVIC(OPEN);
	
	/* 创建任务 */
	AppTaskCreate();
	
	
    /* 启动调度，开始执行任务 */
    vTaskStartScheduler();

	/* 
	  如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
	  heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 30 * 1024 ) )
	*/
	while(1);
}


/*

*********************************************************************************************************
*	函 数 名: vTask_AGV_To_Network
*	功能说明: 	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2  

*********************************************************************************************************

*/

static void vTask_AGV_To_Network(void *pvParameters) //磁导航
{
	TickType_t xLastWakeTime;const TickType_t xFrequency = 5000; //5s
	xLastWakeTime = xTaskGetTickCount();
	
	while (1)
	{
		//AGV_To_Network(&Work_Mode,&Network_Reserved_BUF[0]);    //通过网络发送AGV状态函数
		AGV_To_Network(&Work_Mode,&Network_Reserved_BUF[0]);    //通过网络发送AGV状态函数
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}





/*

*********************************************************************************************************
*	函 数 名: vTask_Right_90Angel_Check
*	功能说明: 	
*	形    参: 检测是否完成右转90度任务
*	返 回 值: 无
*   优 先 级: 2  

*********************************************************************************************************

*/
static void vTask_Right_90Angel_Check(void *pvParameters) 
{
	while(1)
	{
		if(Direction == Forward ) 
		{
			U2_Right_90Angel_Check();
		}
		else if(Direction == Back_Up)
		{
			U5_Right_90Angel_Check();
		}
		vTaskDelay(10); 
	}
}

/*

*********************************************************************************************************
*	函 数 名: vTask_Left_90Angel_Check
*	功能说明: 	
*	形    参: 检测是否完成左转90度
*	返 回 值: 无
*   优 先 级: 2  

*********************************************************************************************************

*/
static void vTask_Left_90Angel_Check(void *pvParameters) 
{
	while(1)
	{
		if(Direction == Forward ) 
		{
			U2_Left_90Angel_Check();
		}
		else if(Direction == Back_Up)
		{
			U5_Left_90Angel_Check();
		}
		vTaskDelay(10); 
	}

}







/*

*********************************************************************************************************
*	函 数 名: vTask_Forward_Barrier
*	功能说明: 超声波障碍物输入   前方	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2  

*********************************************************************************************************

*/

static void vTask_Forward_Barrier(void *pvParameters) 
{
	BaseType_t xResult;
	uint32_t ulValue;
	
	while (1)
	{
		//0x00000000
		xResult = xTaskNotifyWait(0xFFFFFFFF, 
									0xFFFFFFFF, 
									&ulValue, /* 保存 ulNotifiedValue 到变量 ulValue 中 */
									portMAX_DELAY); /* 最大允许延迟时间 */
		if( xResult == pdPASS && Direction == Forward )
		{
			
			if((ulValue & BIT_0) != 0)   
			{
				AGV_Have_Barrier();
			}
			else if((ulValue & BIT_1) != 0)
			{
				AGV_None_Barrier();
			} 
			
		}
		vTaskDelay(10); 
		
	}
}


/*

*********************************************************************************************************
*	函 数 名: vTask_BackUp_Barrier
*	功能说明: 超声波障碍物输入  后方
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2  

*********************************************************************************************************

*/

static void vTask_BackUp_Barrier(void *pvParameters) 
{
	BaseType_t xResult;
	uint32_t ulValue;
	
	while (1)
	{
		xResult = xTaskNotifyWait(0xFFFFFFFF, 
									0xFFFFFFFF, 
									&ulValue, /* 保存 ulNotifiedValue 到变量 ulValue 中 */
									portMAX_DELAY); /* 最大允许延迟时间 */
		
		if( xResult == pdPASS && Direction == Back_Up )
		{
			
			if((ulValue & BIT_0) != 0)   
			{
				AGV_Have_Barrier();
			}
			else if((ulValue & BIT_1) != 0)
			{
				AGV_None_Barrier();
			} 
			
		}
		vTaskDelay(10); 
		
	}
}

/*
*********************************************************************************************************
*	函 数 名: vTask_Manual
*	功能说明: 手动停止按钮
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2  

*********************************************************************************************************

*/

static void vTask_Manual(void *pvParameters) 
{
	BaseType_t xResult;
	uint32_t ulValue;
	
	while (1)
	{
		xResult = xTaskNotifyWait(0x00000000, 
									0xFFFFFFFF, 
									&ulValue, /* 保存 ulNotifiedValue 到变量 ulValue 中 */
									portMAX_DELAY); /* 最大允许延迟时间 */
		if( xResult == pdPASS )
		{
			/* 接收到消息，检测那个位被按下 */
			if((ulValue & BIT_0) != 0)
			{
				AGV_StopBManual();
			}
			else if((ulValue & BIT_1) != 0)
			{
				AGV_RunBManual();
			} 
		}
		vTaskDelay(10);
		
	}
}



/*

*********************************************************************************************************
*	函 数 名: vTask_U2_Navigation
*	功能说明: 串口2 磁导航任务 解析数据	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2  

			磁导航 数据格式（一帧）共5笔数据				
			第1笔	第2笔	第3笔	第4笔	第5笔
			0x55	0x54	0x09	DATA	~DATA
			HEAD1	HEAD2	NUM	传感器数值	取反

串口2磁导航
*********************************************************************************************************

*/
static void vTask_U2_Navigation(void *pvParameters) //磁导航
{

	while (1)
	{
		UART2_Done = 0;
		U2_State( ); //状态机

		if(UART2_Done)
		{
			comClearRxFifo(COM2);
		}
		UART2_Done = 0;
		vTaskDelay(10);       
	}

}


/*
*********************************************************************************************************
*	函 数 名: vTask_U3_RFID
*	功能说明: 串口3 RFID读卡器任务	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 3  
*********************************************************************************************************
*/
static void vTask_U3_RFID(void *pvParameters)//读卡器
{
	
    while(1)
    {
		
		UART3_Done = 0;
		U3_State( ); //状态机
		vTaskDelay(5);
    }
}


/*
*********************************************************************************************************
*	函 数 名: vTask_U5_Navigation
*	功能说明: 串口5 磁导航传感器	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 3  
*********************************************************************************************************
*/

static void vTask_U5_Navigation(void *pvParameters)
{
  	while (1)
	{

		UART5_Done = 0;
		U5_State( ); //状态机
		if(UART5_Done)
		{
			comClearRxFifo(COM5);
		}
		UART5_Done = 0;
		vTaskDelay(10);       
	}

}


/*
*********************************************************************************************************
*	函 数 名: vTask_U6_HMI
*	功能说明: 串口6 HMI人机界面任务	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 3  
*********************************************************************************************************
*/
static void vTask_U6_HMI(void *pvParameters)
{
    while(1)
    {
		U6_State( ); //状态机
		vTaskDelay(20);
    }
}







/*
*********************************************************************************************************
*	函 数 名: vTaskCheck
*	功能说明: 检测外设是否正常
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 8 
*   外设初始化
*********************************************************************************************************
*/
static void vTaskCheck(void *pvParameters)
{
    while(1)
    {
		
		
		vTaskSuspend(xHandle_U2_Navigation); 
		vTaskSuspend(xHandle_U3_RFID);
		bsp_DelayMS(1000);
		
		/*
		AGV联网初始化
		*/

		AGV_To_Network(&Work_Mode,&Network_Reserved_BUF[0]);    //通过网络发送AGV状态函数
		
		bsp_DelayMS(1000);
		/*
		RFID读卡器检测
		*/
		
		//RFID_Check();
		
		
		/*
		磁导航传感器检测，由于是主动上传数据模式
		检测成功后关闭串口2接受中断，
		防止FIFO溢出
		等HMI人机界面下达正确指令后，重新允许接受中断
		*/
		
		
		U2_Navigation_Check();
		
		/*
		磁导航传感器检测，由于是主动上传数据模式
		检测成功后关闭串口5接受中断，
		防止FIFO溢出
		等HMI人机界面下达正确指令后，重新允许接受中断
		*/
		RFID_Check();


	    //U5_Navigation_Check();
		//USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); 
		
		
		/*
		超声波传感器检测，由于是主动上传数据模式
		检测成功后关闭串口5接受中断，
		防止FIFO溢出
		等HMI人机界面下达正确指令后，重新允许接受中断
		*/
		//Attitude_Check();
		//USART_ITConfig(UART5,USART_IT_RXNE,DISABLE);

		/*
		伺服电机检测，485MODBUS协议
		*/
		Servo_Init();
	
		printf("调试窗口.AGV连接.val=1");
		bsp_DelayMS(100);
		HMI_END();
		bsp_DelayMS(1000);
		
		///检测任务执行完毕后，删除检测任务，不再执行此任务
		////关闭串口接受中断，并且清除FIFO的数据，以便正常运行
		USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); //关闭串口2的磁导航串口

		
		//默认挂起串口5
		vTaskSuspend(xHandle_U5_Navigation);  //挂起串口5 磁导航
		
		USART_ITConfig(UART5,USART_IT_RXNE,DISABLE); //关闭串口5的磁导航串口
		

		////关闭串口接受中断，并且清除FIFO的数据，以便正常运行
		comClearRxFifo(COM2);
		comClearRxFifo(COM3);
		comClearRxFifo(COM5);
			
	
		///任务挂起，直到HMI收到指令才恢复任务	
		vTaskSuspend(xHandle_AGV_Action); 
		vTaskSuspend(xHandle_Left_90Angel_Check);
		vTaskSuspend(xHandle_Right_90Angel_Check);
		//vTaskSuspend(xHandle_BackUp_Barrier);
		vTaskSuspend(xHandlevTaskCheck);
	}
	
}





/*

*********************************************************************************************************
*	函 数 名: vTask_AGV_Action
*	功能说明: AGV动作任务,一直处于阻塞状态等待磁传感器信号，并根据信号动作	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级:   

*********************************************************************************************************

*/

static void vTask_AGV_Action(void *pvParameters)
{
	

	BaseType_t Navigation;
	uint32_t ulValue;
	
	while(1)
	{		
		//设置portMAX_DELAY,让任务一直处于阻塞状态，一旦接到到磁导航传感器信号，就马上工作，实现实时性
		
		
		//第一个参数  如果是0xFFFFFFFF 使用通知之前，清零
		//第二个参数  如果是0xFFFFFFFF 在函数xTaskNotifyWait()退出前，清零
		// 0x00000000
		Navigation = xTaskNotifyWait(0xFFFFFFFF,      
						          0xFFFFFFFF,      
						          &ulValue,        /* 保存ulNotifiedValue到变量ulValue中 */
						          portMAX_DELAY);  /* 最大允许延迟时间 */
		
		if( Navigation == pdPASS ) 
		{
			/*  
				接收到消息，检测AGV需要执行的动作 
			    设置成if else if为了实现互锁，停车—执行—右转—左转的互锁
			    AGV 停车
			
				对于舵轮结构来说
				当出现分岔口的时候，由于if else if 逻辑，
				小车会优先右转，左转，最后才是直行，
				这样可以是实现简单的路径选择
				如果后续要增加功能的话，可以在分岔口前面设置IC卡，小车提前减速和选择路径
			
				对于双轮差速结构，转弯是原地旋转
				
			
			*/
			/* AGV 脱轨 */
			if((ulValue & BIT_4) != 0 ) 
			{
				Go_Straight();
				/* 
				正常运行时绿灯亮
				*/
				Running_Green_Light();			
				if(Derail)
				{
					bsp_DelayMS(100);
					printf("行驶界面.脱轨.val=0"); //重新回到轨道
					bsp_DelayMS(100);
					HMI_END();
					Derail = 0;
				}	
			}
			else if((ulValue & BIT_0) != 0 ) 
			{
				Stop();
				bsp_DelayMS(100);
				printf("行驶界面.脱轨.val=1"); //脱轨
				bsp_DelayMS(100);
				HMI_END();
				Derail = 1;
				/* 
				脱轨时报警红灯亮
				*/
				Alarm_Red_Light();
				
			}		
			/* AGV 右转  */
			else if((ulValue & BIT_1) != 0)
			{
				Turn_Right(&U2_buf[0]);
				/* 
				正常运行时绿灯亮
				*/
				Running_Green_Light();			
				if(Derail)
				{
					bsp_DelayMS(100);
					printf("行驶界面.脱轨.val=0"); //重新回到轨道
					bsp_DelayMS(100);
					HMI_END();
					Derail = 0;
				}
			}
			/* AGV 左转  */
			else if((ulValue & BIT_2) != 0) 
			{
				Turn_Left(&U2_buf[0]);
				/* 
				正常运行时绿灯亮
				*/				
				Running_Green_Light();
				if(Derail)
				{
					bsp_DelayMS(100);
					printf("行驶界面.脱轨.val=0"); //重新回到轨道
					bsp_DelayMS(100);
					HMI_END();
					Derail = 0;
				}
			}
			/* AGV 直行 */
			else if((ulValue & BIT_3) != 0)
			{

				Go_Straight();
				/* 
				正常运行时绿灯亮
				*/
				Running_Green_Light();
				if(Derail)
				{
					bsp_DelayMS(100);
					printf("行驶界面.脱轨.val=0"); //重新回到轨道
					bsp_DelayMS(100);
					HMI_END();
					Derail = 0;
				}
			}	
		}	
	}	
}


/*
*********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参：无
*	返 回 值: 无


创建任务
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
	taskENTER_CRITICAL();           //进入临界区
	
/////发送AGV状态任务
	xTaskCreate( vTask_AGV_To_Network,   	/* 任务函数  */
                 "vTask_AGV_To_Network",     	/* 任务名    */
                 512,               	/* 任务栈大小，单位word，也就是4字节 */
                 NULL,              	/* 任务参数  */
                 1,                 	/* 任务优先级*/
                 &xHandle_AGV_To_Network  );  /* 任务句柄  */
	
/////串口2 磁导航传感器数据处理任务	
	xTaskCreate( vTask_U2_Navigation,   	/* 任务函数  */
                 "vTask_U2_Navigation",     	/* 任务名    */
                 512,               	/* 任务栈大小，单位word，也就是4字节 */
                 NULL,              	/* 任务参数  */
                 2,                 	/* 任务优先级*/
                 &xHandle_U2_Navigation  );  /* 任务句柄  */
	
/////串口3 RFID读卡器数据处理任务	
	xTaskCreate( vTask_U3_RFID,    		/* 任务函数  */
                 "vTask_U3_RFID",  		/* 任务名    */
                 512,         		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,        		/* 任务参数  */
				 7,           		/* 任务优先级*/
                 &xHandle_U3_RFID  ); /* 任务句柄  */
				 
/////串口5 磁导航传感器数据处理任务	
	xTaskCreate( vTask_U5_Navigation,     		/* 任务函数  */
                 "vTask_U5_Navigation",   		/* 任务名    */
                 512,             		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		/* 任务参数  */
                 2,               		/* 任务优先级*/
                 &xHandle_U5_Navigation);  /* 任务句柄  */
				 
/////串口6 触摸屏数据处理任务				 
	xTaskCreate( vTask_U6_HMI,     		/* 任务函数  */
                 "vTask_U6_HMI",   		/* 任务名    */
                 512,             		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		/* 任务参数  */
                 5,               		/* 任务优先级*/
                 &xHandle_U6_HMI   );  /* 任务句柄  */				 
				 
/////外设检测任务	
	xTaskCreate( vTaskCheck,     		/* 任务函数  */
				 "vTaskCheck",   		/* 任务名    */
				 512,            		/* 任务栈大小，单位word，也就是4字节 */
				 NULL,           		/* 任务参数  */
				 8,              		/* 任务优先级*/
				 &xHandlevTaskCheck );   /* 任务句柄  */		 
				 
			 
				 
				 
/////障碍物任务	 前方		
	xTaskCreate( vTask_Forward_Barrier,     		/* 任务函数  */
				 "vTask_Forward_Barrier",		/* 任务名    */
				 512,            		/* 任务栈大小，单位word，也就是4字节 */
				 NULL,           		/* 任务参数  */
				 6,              		/* 任务优先级*/
				 &xHandle_Forward_Barrier   );   /* 任务句柄  */		
				 
				 
/////障碍物任务	 后方		
	xTaskCreate( vTask_BackUp_Barrier,     		/* 任务函数  */
				 "vTask_BackUp_Barrier",		/* 任务名    */
				 512,            		/* 任务栈大小，单位word，也就是4字节 */
				 NULL,           		/* 任务参数  */
				 6,              		/* 任务优先级*/
				 &xHandle_BackUp_Barrier     );   /* 任务句柄  */					 
				 
				 
				 
/////手动停车、启动任务					 
	xTaskCreate( vTask_Manual,     		/* 任务函数  */
				 "vTask_Manual",		/* 任务名    */
				 512,            		/* 任务栈大小，单位word，也就是4字节 */
				 NULL,           		/* 任务参数  */
				 5,              		/* 任务优先级*/
				 &xHandle_Manual );   /* 任务句柄  */				 
				 
				 

////////////AGV动作任务  
//	xTaskCreate( vTask_AGV_Action,     		/* 任务函数  */
//				 "vTask_AGV_Stop",   		/* 任务名    */
//				 512,            		/* 任务栈大小，单位word，也就是4字节 */
//				 NULL,           		/* 任务参数  */
//				 7,              		/* 任务优先级*/
//				 &xHandle_AGV_Action);   /* 任务句柄  */
				 
	xTaskCreate( vTask_AGV_Action,     		/* 任务函数  */
				 "vTask_AGV_Stop",   		/* 任务名    */
				 512,            		/* 任务栈大小，单位word，也就是4字节 */
				 NULL,           		/* 任务参数  */
				 4,              		/* 任务优先级*/
				 &xHandle_AGV_Action);   /* 任务句柄  */		
		 
				 
////////////AGV左转90度检测任务 			 
	xTaskCreate( vTask_Left_90Angel_Check,     		/* 任务函数  */
				 "vTask_Left_90Angel_Check",   		/* 任务名    */
				 512,            		/* 任务栈大小，单位word，也就是4字节 */
				 NULL,           		/* 任务参数  */
				 3,              		/* 任务优先级*/
				 &xHandle_Left_90Angel_Check );   /* 任务句柄  */

////////////AGV右转90度检测任务
	xTaskCreate( vTask_Right_90Angel_Check,     		/* 任务函数  */
				 "vTask_Right_90Angel_Check",   		/* 任务名    */
				 512,            		/* 任务栈大小，单位word，也就是4字节 */
				 NULL,           		/* 任务参数  */
				 3,              		/* 任务优先级*/
				 &xHandle_Right_90Angel_Check );   /* 任务句柄  */			 		 
				 
											 
	taskEXIT_CRITICAL();            //退出临界区							 
								 
								 
}


/*********************************************************************************************************
*	函 数 名: Servo_Init
*	功能说明: 初始化2个伺服电机：1.Modbus 使能 2.驱动器输出使能 3.读取报警代码
*	形    参: Pulse
*	返 回 值: 无
*   地址 参数名称 		只读/读写 	     参数范围 			参数说明
     0   Modbus 使能       读写 			    0~1             0：modbus 禁止
															1：modbus 使能
															
	 1	 驱动器输出使能     读写			    0~1	            0：驱动器输出禁止
															1：驱动器输出使能
	
	 2   电机目标速度       读写           0~3000 r/min      速度模式时，目标速度
															位置模式时，最大速度
															
	 3   电机加速度         读写           0~30000(r/min)/s   电机加速度																


*   
*********************************************************************************************************/
void Servo_Init(void)
{
	//1.写 驱动伺服器  从机地址1（功能码 06 寄存器地址 0x00  Modbus 使能） 1
	//2.写 转向伺服器  从机地址2（功能码 06 寄存器地址 0x00  Modbus 使能） 1
	//MODH_WriteParam_06H(1,0,1);   

	printf("调试窗口.显示区.txt=\"伺服电机驱动器初始化中\"");
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(500);

	while(MODH_WriteParam_06H(1,0,1) != 1)
	{
		printf("调试窗口.显示区.txt=\"左1号伺服电机Modbus使能中\"");
		bsp_DelayMS(100);
		HMI_END();
		bsp_DelayMS(100);

	}
	bsp_DelayMS(500);
	printf("调试窗口.显示区.txt=\"左1号伺服电机Modbus使能成功\"");
	bsp_DelayMS(100);
	HMI_END();

	
	
	
	while(MODH_WriteParam_06H(2,0,1)!=1)
	{
		printf("调试窗口.显示区.txt=\"右2号伺服电机Modbus使能中\"");
		bsp_DelayMS(100);
		HMI_END();
		bsp_DelayMS(100);

	}
	printf("调试窗口.显示区.txt=\"右2号伺服电机Modbus使能成功\"");
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(1000);
	
}



/*
被调用的函数
*/
/*********************************************************************************************************
*	函 数 名: U2_State
*	功能说明: 磁传感器上传的数据包头验证状态机
*	形    参: Pulse
*	返 回 值: 无   



十六路磁导航数据：55 54 data1 data2 CRC
data1 01~08
data1 09~16
*********************************************************************************************************/
void U2_State(void)
{
	//const uint8_t CDH_HEAD[3]= {0x55,0x54,0x09}; //返回数据的包头 5个字节 八路磁导航
	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //返回数据的包头 5个字节    十六路磁导航
	
	if (comGetChar(COM2, &U2_read))
	{
		switch (U2_ucStatus)
		{
			/* 状态0保证接收到0x01 */
			case 0:
			{
				if(U2_read == CDH_HEAD[U2_ucStatus])
				{
					U2_ucStatus = 1;   
					//printf("0x55\r\n");
					//ucCount++;
				}
				break;
			}                  

			case 1:
			{
				if(U2_read == CDH_HEAD[U2_ucStatus])
				{
					U2_ucStatus = 2;
					//printf("0x54\r\n");
					//ucCount++;
					U2_Navigation_Correct =1;
				}
				else
				{
					U2_ucStatus = 0;
					//ucCount  = 0;
				}
				break;
			}  

			case 2:
			{
				/*
					data1 data2 CRC  CRC =  data1 + data2
				*/
				U2_buf[U2_ucCount] = U2_read; 
				if(U2_ucCount==2)
				{
					if(U2_buf[2] == (U2_buf[0]+U2_buf[1]))
					{
						U2_Deal(&U2_buf[0]);
						U2_ucStatus = 0;
						U2_ucCount=0;
						UART2_Done = 1;	
					}
					U2_ucStatus = 0;
					U2_ucCount=0;
						
				}
				else
				{
					U2_ucCount++;
				}
				break;	
			}
		   default:
				break;
		}
	}
}

/*********************************************************************************************************
*	函 数 名: U2_State_Check
*	功能说明: 磁传感器上传的数据包头验证状态机，用于开机磁传感器检测
*	形    参: Pulse
*	返 回 值: 无   
*********************************************************************************************************/
void U2_State_Check(void)
{
	//const uint8_t CDH_HEAD[3]= {0x55,0x54,0x09}; //返回数据的包头 5个字节  八路磁导航
	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //返回数据的包头 5个字节 十六路磁导航
	if (comGetChar(COM2, &U2_read))
	{
		switch (U2_ucStatus_Check)
		{
			/* 状态0保证接收到0x01 */
			case 0:
			{
				if(U2_read == CDH_HEAD[U2_ucStatus_Check])
				{
					U2_ucStatus_Check = 1;   				
				}
				else
				{
					U2_ucStatus_Check = 0;
				}
				break;
			}                  

			case 1:
			{
				if(U2_read == CDH_HEAD[U2_ucStatus_Check])
				{
					
					bsp_DelayMS(300);
					HMI_END();
					bsp_DelayMS(100);
					U2_ucStatus_Check = 3;
					U2_Navigation_Correct =1;
				}
				else
				{
					U2_ucStatus_Check = 0;
				}
				break;
			}  
			default:
			{
				U2_ucStatus_Check = 0;
				
				break;
			}
			
	
		}
	}
}


/*********************************************************************************************************
*	函 数 名: U2_Deal
*	功能说明: 根据磁传感器探头检测数据发送对应的信号量给 vTask_AGV_Action
*	形    参: uint8_t* data   
*	返 回 值: 无   
	AGV正常工作
*********************************************************************************************************/
void U2_Deal(uint8_t* data)
{	
	int data1_num = 0;
	int data2_num = 0;;
	uint8_t data1 = *data;
	uint8_t data2 = *(data+1);
	/*
		*data 		01~08
		*(data+1)	09~16
	
		当01~05探头被触发，右转
		
		当12~16探头被触发，左转
	*/
	/*
		统计磁传感器检测到磁条的探头数
	*/
	//检测数据中含1个数的算法
	while(data1)
	{
		data1 = data1&(data1-data1_num-1);
		data1_num++;
	}
	
	while(data2)
	{
		data2 = data2&(data2-data2_num-1);
		data2_num++;
	}
	/* 
		为了十字路口处转弯 
		磁导航感应到的探头数大于7，则直行
	*/
	
	if((data1_num + data2_num) >=7)
	{
		if(Guidance_State != Auto_Crossing)
		{	
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_4,             /* 设置目标任务事件标志位bit0  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_0进行或操作， 
														  将结果赋值给事件标志位。*/
			Guidance_State = Auto_Crossing;	
			State = Moving; //行驶状态
		}	
	}

	/* 脱轨 */
	else if(*data == 0x00 & *(data+1) ==0x00)//判断脱离轨道，马上停车 0000 0000 
	{
		if(Guidance_State != Auto_Stop)
		{	
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_0,             /* 设置目标任务事件标志位bit0  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_0进行或操作， 
														  将结果赋值给事件标志位。*/
			Guidance_State = Auto_Stop;	
			State = Derail_ERR; //脱轨状态
		}	
	}
	
	
	/* 右转 */
	else if((*data & 0xFC))  //放大是 0xF0=1111 0000  中间是0xF8 =1111 1000  再缩小就是 0xFC = 1111 1100
	{
		//if(Guidance_State != Auto_Right)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_1,             /* 设置目标任务事件标志位bit1  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_1进行或操作， 
														  将结果赋值给事件标志位。*/
			Guidance_State = Auto_Right;
			State = Moving; //行驶状态
		}
	}
	
	/* 左转 */
	else if(*(data+1) & 0x3F) //放大是 0x0F=0000 1111 中间是0x1F =0001 1111 再缩小就是 0x3F = 0011 1111
	{
		//if(Guidance_State != Auto_Left)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_2,             /* 设置目标任务事件标志位bit2  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_2进行或操作， 
														  将结果赋值给事件标志位。*/
			Guidance_State = Auto_Left;
			State = Moving; //行驶状态
		}		
	}
	
	/* 直行 */
	else   
	{
		if(Guidance_State != Auto_Straight)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_3,             /* 设置目标任务事件标志位bit3  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_3进行或操作， 
														  将结果赋值给事件标志位。*/
				
			Guidance_State = Auto_Straight;
			State = Moving; //行驶状态
		}

	}	
	
	
}


/*********************************************************************************************************
*	函 数 名: U3_State
*	功能说明: RFID上传的数据包头验证状态机
*	形    参:   
*	返 回 值: 无   
*********************************************************************************************************/
void U3_State(void) //RFID
{

	const uint8_t RFID_HEAD[5]= {0x04,0x16,0x03,0x20,0x00}; //返回数据的包头 5个字节
	if (comGetChar(COM3, &U3_read))
	{
		switch (U3_ucStatus)
		{
			/* 状态0保证接收到0x04 */
			case 0:
			{

				if(U3_read == RFID_HEAD[U3_ucStatus])
				{
					U3_ucStatus = 1;   

				}
				else
				{
					U3_ucStatus = 0;

				}				
				break;
			}                  
			/* 状态1保证接收到0x16 */
			case 1:
			{

				if(U3_read == RFID_HEAD[U3_ucStatus])
				{
					U3_ucStatus = 2;   

				}
				else
				{
					U3_ucStatus = 0;
					//ucCount  = 0;
				}							
				break;
			}  
			/* 状态1保证接收到0x03 */
			case 2:
			{

				if(U3_read == RFID_HEAD[U3_ucStatus])
				{
					U3_ucStatus = 3;   

				}
				else
				{
					U3_ucStatus = 0;
					//ucCount  = 0;
				}							
				break;
			} 
			/* 状态1保证接收到0x20 */	
			case 3:
			{
				//printf("%X\r\n",U3_read);
				if(U3_read == RFID_HEAD[U3_ucStatus])
				{
					U3_ucStatus = 4;   

				}
				else
				{
					U3_ucStatus = 0;
					//ucCount  = 0;
				}							
				break;
			}
			/* 状态1保证接收到0x00 */	
			case 4:
			{
				if(U3_read == RFID_HEAD[U3_ucStatus])
				{
					U3_ucStatus = 5;   
				}
				else
				{
					U3_ucStatus = 0;

				}							
				break;
			}
			
			case 5:
			{

				U3_buf[U3_ucCount] = U3_read;              
				 /* 接收够3个数据 */
				if(U3_ucCount == 3-1)
				{	
					U3_Deal(&U3_buf[0]);
					U3_ucStatus = 0;
					U3_ucCount=0;
					UART3_Done = 1;
					Discern_IC_Card();
				}
				else
				{
					U3_ucCount++;
				}
				break;				

			}
			default:
				break;
		}
	}
}

/*********************************************************************************************************
*	函 数 名: U3_Deal
*	功能说明: 把地址给RFID_IN
*	形    参: uint8_t* data   
*	返 回 值: 无   
*********************************************************************************************************/
void U3_Deal(uint8_t* data)
{
	//uint8_t *RFID_IN[20];
	RFID_IN = data; //把地址给RFID_IN
	
}


/*********************************************************************************************************
*	函 数 名: Verify_RFID
*	功能说明: 根据RFID读卡器读到的卡片数据与HMI的目的地指令进行判断，判断当前地点是否为目的地
*	形    参: uint8_t* data ，uint8_t* RFID  
*	返 回 值: 无   
*********************************************************************************************************/
uint8_t Verify_RFID(uint8_t* data,uint8_t* RFID)
{
	int i;
	for(i = 0;i <3;i++)
	{
		if(*data != *RFID) //验证地点
		{
			PLACE_Match=0; //不是指定目的地	
			break;
		}
		else
		{
			data++;
			RFID++;
			PLACE_Match=1;//是指定目的地	
		}
	}	
	if(PLACE_Match)
	{

		/*
			停车等待时候黄色灯亮
		*/
		Waiting_Yellow_Light();
	}
	return PLACE_Match;

}
/*********************************************************************************************************
*	函 数 名: U5_State
*	功能说明: 串口5 磁导航传感器的数据包头验证状态机
*	形    参:   
*	返 回 值: 无   
*********************************************************************************************************/
void U5_State(void)
{
	//const uint8_t CDH_HEAD[3]= {0x55,0x54,0x09}; //返回数据的包头 5个字节 八路磁导航
	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //返回数据的包头 5个字节    十六路磁导航
	
	if (comGetChar(COM5, &U5_read))
	{
		switch (U5_ucStatus)
		{
			/* 状态0保证接收到0x01 */
			case 0:
			{
				if(U5_read == CDH_HEAD[U5_ucStatus])
				{
					U5_ucStatus = 1;   

				}
				break;
			}                  

			case 1:
			{
				if(U5_read == CDH_HEAD[U5_ucStatus])
				{
					U5_ucStatus = 2;
					U5_Navigation_Correct =1;
				}
				else
				{
					U5_ucStatus = 0;

				}
				break;
			}  

			case 2:
			{
				if(U5_read!=0x55) ///这里有个BUG,第一笔数据为0X55
				{
					U5_buf[U5_ucCount] = U5_read;              
					 /* 接收够2个数据 */
					if(U5_ucCount == 2-1)
					{	
	
						
						U5_Deal(&U5_buf[0]);
						U5_ucStatus = 0;
						U5_ucCount=0;
						UART5_Done = 1;
					}
					else
					{
						U5_ucCount++;
					}
				}
				else
				{
					U5_ucStatus = 0;
					U5_ucCount=0;
				}
				break;	
			}
		   default:
				break;
		}
	}
}
/*********************************************************************************************************
*	函 数 名: U5_Deal
*	功能说明: 根据姿态传感器检测到的数据进行判断，对Z轴角度的获取和判断
*	返 回 值: 无   
*********************************************************************************************************/

void U5_Deal(uint8_t* data)
{
	int data1_num = 0;
	int data2_num = 0;;
	uint8_t data1 = *data;
	uint8_t data2 = *(data+1);
	/*
		*data 		01~08
		*(data+1)	09~16
	
		当01~05探头被触发，右转
		
		当12~16探头被触发，左转
	*/
	/*
		统计磁传感器检测到磁条的探头数
	*/
	while(data1)
	{
		data1 = data1&(data1-data1_num-1);
		data1_num++;
	}
	
	while(data2)
	{
		data2 = data2&(data2-data2_num-1);
		data2_num++;
	}
	/* 
		为了十字路口处转弯 
		磁导航感应到的探头数大于7，则直行
	*/
	
	if((data1_num + data2_num) >=7)
	{
		if(Guidance_State != Auto_Crossing)
		{	
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_4,             /* 设置目标任务事件标志位bit0  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_0进行或操作， 
														  将结果赋值给事件标志位。*/
			Guidance_State = Auto_Crossing;	
			State = Moving; //行驶状态
		}	
	}

	/* 脱轨 */
	else if(*data == 0x00 & *(data+1) ==0x00)//判断脱离轨道，马上停车 0000 0000 或者 到站，停车 1111 1111
	{
		if(Guidance_State != Auto_Stop)
		{	
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_0,             /* 设置目标任务事件标志位bit0  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_0进行或操作， 
														  将结果赋值给事件标志位。*/
			Guidance_State = Auto_Stop;	
			State = Derail_ERR; //脱轨状态
		}	
	}
	
	
	/* 右转 */
	else if((*data & 0xF8))    
	{
		if(Guidance_State != Auto_Right)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_1,             /* 设置目标任务事件标志位bit1  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_1进行或操作， 
														  将结果赋值给事件标志位。*/
			Guidance_State = Auto_Right;
			State = Moving; //行驶状态
		}
	}
	
	/* 左转 */
	else if(*(data+1) & 0x1F)    	
	{
		if(Guidance_State != Auto_Left)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_2,             /* 设置目标任务事件标志位bit2  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_2进行或操作， 
														  将结果赋值给事件标志位。*/
			Guidance_State = Auto_Left;
			State = Moving; //行驶状态
		}		
	}
	
	/* 直行 */
	else   
	{
		if(Guidance_State != Auto_Straight)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* 目标任务 */
									BIT_3,             /* 设置目标任务事件标志位bit3  */
									eSetBits);         /* 将目标任务的事件标志位与BIT_3进行或操作， 
														  将结果赋值给事件标志位。*/
				
			Guidance_State = Auto_Straight;
			State = Moving; //行驶状态
		}
				
		
	}	
	
}

	
	
	
	


/*********************************************************************************************************
*	函 数 名: U6_State
*	功能说明: HMI触摸屏上传的数据包头验证状态机
*	形    参:   
*	返 回 值: 无   
*********************************************************************************************************/
void U6_State(void)
{
	const uint8_t HMI_HEAD[2]={0x55,0x54};//HMI数据包头 
	if (comGetChar(COM6, &U6_read))
	{
		switch (U6_ucStatus)
		{
			/* 状态0保证接收到0x04 */
			case 0:
			{
				U6_ucStatus = 0;
				if(U6_read == HMI_HEAD[U6_ucStatus])
				{
					U6_ucStatus = 1;   

				}
				else
				{
					U6_ucStatus = 0;

				}	
				break;
			}                  
			 /* 状态1保证接收到0x16 */
			case 1:	
			{
				if(U6_read == HMI_HEAD[U6_ucStatus])
				{
					U6_ucStatus = 2;   

				}
				else
				{
					U6_ucStatus = 0;

				}	
				break;
			}
			case 2:
			{
				U6_buf[U6_ucCount] = U6_read;              
				 /* 接收够24个数据 

				*/
				if(U6_ucCount == 24-1)
				{	
					if(U6_read == 0xFF) //验证包尾
					{
						U6_Deal(&U6_buf[0]);
					}
					U6_ucStatus = 0;
					U6_ucCount=0;
				}
				else
				{
					U6_ucCount++;
				}
				break;				
			}
			default:
				break;	
		}
	}
}



/*********************************************************************************************************
*	函 数 名: U6_Deal
*	功能说明: 根据HMI触摸屏上传的数据进行处理
*	返 回 值: 无   
*********************************************************************************************************/
void U6_Deal(uint8_t* data)
{
/*
*(data)   功能码
	
*(data+1)	A1      *(data+6)	B1		*(data+11)	C1		*(data+16)	D1		*(data+21)	终点
*(data+2)	A2      *(data+7)	B2		*(data+12)	C2		*(data+17)	D2		*(data+22)	起点
*(data+3)	A3		*(data+8)	B3		*(data+13)	C3		*(data+18)	D3
*(data+4)	A4		*(data+9)	B4		*(data+14)	C4		*(data+19)	D4
*(data+5)	A5		*(data+10)	B5		*(data+15)	C5		*(data+20)	D5
*/
//	USART_ITConfig(UART5,USART_IT_RXNE,DISABLE); //关闭超声波避障串口
//	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); //关闭磁导航串口
	
	uint8_t place_i;
	uint8_t ID_i;
	uint8_t HMI_Straight_Speed_L;
	uint8_t HMI_Straight_Speed_H;
	switch(*data) //功能码
	{
		case 0xF1: //上位机手动模式
		{
//			Manual_Place = 1; //上位机手动指定地点模式
//			Auto_Place = 0;	
			Work_Mode = Manual_Mode;
			printf("收到手动模式.val=1");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);
			
			for(place_i=0;place_i<26;place_i++)
			{
				HMI_Command[place_i]=*(data+1);
				//printf("HMI:%X",*(data+1));
				data++;
				//printf("数据：%x\r\n",HMI_Command[place_i]);
			}
			
			PLACE_Gone = 0;
			while(HMI_Command[PLACE_Gone]==0x00)	//剔除不去的目的地
			{
				PLACE_Gone++;
			}
			
			/*
				传递目的地
			*/
			Place = HMI_Command[PLACE_Gone];
			
			/*
				恢复任务
			*/
			Task_Resume();
			
			break;
		}
		case 0xF2://上位机自动模式
		{
//			Auto_Place =1; //上位机自动指定地点模式
//			Manual_Place = 0;
			Work_Mode = Auto_Mode;
			printf("收到自动模式.val=1");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);
			for(place_i=0;place_i<26;place_i++)
			{
				HMI_Command[place_i]=*(data+1);
				data++;
			}	
			
			PLACE_Gone = 0;
			while(HMI_Command[PLACE_Gone]==0x00)	//剔除不去的目的地
			{
				PLACE_Gone++;
			}
			
			/*
				传递目的地
			*/
			Place = HMI_Command[PLACE_Gone];
			/*
				恢复任务
			*/
			Task_Resume();
						
			break;		
		}			
		
		case 0xF3://停车时间到，继续工作时间 
		{
			
			Place_Stop=0; //继续工作标志位
			bsp_DelayMS(300);
			printf("停车时间结束.val=1");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);
			while(HMI_Command[PLACE_Gone]==0x00)	//剔除不去的目的地
			{
				PLACE_Gone++;
			}
			
			/*
				传递目的地
			*/
			Place = HMI_Command[PLACE_Gone];
			
			/*定点停站后，挂起 vTask_U3_RFID
			  直到HMI下达停车时间结束指令才恢复
			*/

//			vTaskResume(xHandle_U2_Navigation);
//			vTaskResume(xHandle_U5_Attitude);
//			vTaskResume(xHandle_U3_RFID);
			/*
				恢复任务
			*/
			Task_Resume();
				
			break;
		}
		case 0xF4://所有目的地到达，进入停车待机模式 
		{
			
			
			Work_Mode = 0 ;//
			State  = Standby ;//默认处于待机状态
			Place = 0;
			bsp_DelayMS(300);
			printf("进入待机模式.val=1");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);

			USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); //串口允许接受USART_IT_RXNE的中断信号 磁导航
			//USART_ITConfig(USART3,USART_IT_RXNE,ENABLE); //串口允许接受USART_IT_RXNE的中断信号 RFID
			USART_ITConfig(UART5, USART_IT_RXNE,DISABLE);  //串口允许接受USART_IT_RXNE的中断信号 超声波

	
			break;
		}
		case 0xF6://HMI发送AGV ID
		{
			
			
			Work_Mode = 0 ;//
			State  = Standby ;//默认处于待机状态
			Place = 0;
//			bsp_DelayMS(300);
//			printf("进入待机模式.val=1");
//			bsp_DelayMS(300);
//			HMI_END();
//			bsp_DelayMS(100);
		    /*
				获取电机 速度
			*/
			for(ID_i=0;ID_i<3;ID_i++)
			{
				Network_AGVID_BUF[ID_i]=*(data+1+ID_i);
			}
			
			break;
		}
		case 0xF7://HMI发送电机速度
		{
			
			
			Work_Mode = 0 ;//
			State  = Standby ;//默认处于待机状态
			Place = 0;
			
			/*
			直行速度      	2字节
			直行修正速度		1字节
			左右转90度速度	1字节
			*/
			
			HMI_Straight_Speed_L = *(data+1); //直行速度的低位
			HMI_Straight_Speed_H = *(data+2); //直行速度的高位
			HMI_Straight_Speed = HMI_Straight_Speed_H<<8+HMI_Straight_Speed_L; //直行速度
			
			HMI_Correction_Speed = *(data+3); //直行修正速度
			HMI_Turn_90_Speed = *(data+4); //左右转90度速度
			
			/*
				只有HMI传进来的速度大于0，才有效，否则采用默认值
			*/
			if(HMI_Straight_Speed >0)
			{
				Speed_Straight = HMI_Straight_Speed;
			}

			if(HMI_Correction_Speed >0)
			{
				Speed_Difference =HMI_Correction_Speed;
			}

			if(HMI_Turn_90_Speed >0)
			{
				Speed_90Angle = HMI_Turn_90_Speed;
			}

			break;
		}
	}	
	//UART6_Match = 0;
	//HMI_First=1;
	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE); //串口允许接受USART_IT_RXNE的中断信号 磁导航
	//USART_ITConfig(USART3,USART_IT_RXNE,ENABLE); //串口允许接受USART_IT_RXNE的中断信号 RFID
	USART_ITConfig(UART5, USART_IT_RXNE,ENABLE);  //串口允许接受USART_IT_RXNE的中断信号 超声波
	
	////标志位复原
	Barrier	= 0; 		//障碍物标志位
	Guidance_State = 0;	//磁导航标志位



}


/*********************************************************************************************************
*	函 数 名: Discern_IC_Card
*	功能说明: 判断当前读卡点是否为目的地 读IC卡
*	返 回 值: 无   
*********************************************************************************************************/
void Discern_IC_Card(void)	
{
	/*
	分岔路
	*/
	
	while(HMI_Command[PLACE_Gone]==0x00)	//剔除不去的目的地
	{
		PLACE_Gone++;
	}
	
	/*
		传递目的地
	*/
	Place = HMI_Command[PLACE_Gone];
	/*
	
	*/
	if(Verify_RFID(&RFID_IN[0],&RFID_Fork[0])) 
	{
		/*
		输入目的地，根据目的地，给出分岔路选择
		在分岔路，读到分岔路的卡，开始动作
		*/
		switch(Path_Choice(Place))
		{
			case Straight_Flag:
			{
				Fork_Straight();
				break;
			}
			case Left_Flag:
			{
				Fork_Left();
				break;
			}
			case Right_Flag:
			{
				Fork_Right();
				break;
			}
			case Back_Flag:
			{
				Fork_Back();
				break;
			}
			default:
				break;
		 } 
	}
//	else if(Verify_RFID(&RFID_IN[0],&RFID_Forward[0])) 
//	{
//		/*
//		停车，变成前进方向
//		*/
//		Stop();
//		Direction = Forward;
//		Guidance_State = Auto_Stop;


		
		/*
		 前进方向使用PE1外部中断，超声波传感器 1~4号探头
		*/

		
//		if(state ==1)
//		{
//			state = 0;
//			vTaskSuspend(xHandle_BackUp_Barrier);
//			vTaskResume(xHandle_Forward_Barrier);   
//		}

		/*
		 前进方向串口2的磁传感器进行导航
		*/	
//		vTaskSuspend(xHandle_U5_Navigation);  //挂起串口5 磁导航
//		USART_ITConfig(UART5,USART_IT_RXNE,DISABLE);//关闭串口5中断
//		vTaskResume(xHandle_U2_Navigation);   //恢复串口2 磁导航
//		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//打开串口2中断

//		bsp_DelayMS(1000);
//	
//	}
//	else if(Verify_RFID(&RFID_IN[0],&RFID_Back_Up[0])) 
//	{
//		/*
//		停车，变成后退
//		*/
//		
//		Stop();
//		Direction = Back_Up;
//		Guidance_State = Auto_Stop;
		
//		PE1_IN2_NVIC(CLOSE);
//		PE2_IN3_NVIC(OPEN);
		/*
		 后退方向使用PE2外部中断，超声波传感器 5~8号探头
		*/
//		state =1;
//		vTaskSuspend(xHandle_Forward_Barrier);   
//		vTaskResume(xHandle_BackUp_Barrier);
//		
		/*
		 后退方向使用串口5的磁传感器进行导航
		*/
//		vTaskResume(xHandle_U2_Navigation);   //恢复串口2 磁导航
//		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//打开串口2中断
		
//		vTaskSuspend(xHandle_U2_Navigation);  //挂起串口2 磁导航
//		USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);//关闭串口2中断
//		vTaskResume(xHandle_U5_Navigation);   //恢复串口5 磁导航
//		USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);//打开串口5中断
//		//vTaskDelay(1000);
//		bsp_DelayMS(1000);
//	
//	}
//	
	
//	else if(Verify_RFID(&RFID_IN[0],&RFID_90Left[0])) 
//	{
//		/*
//		停车，然后挂起任务，左转，恢复左转90度检测任务
//		
//		*/
//		//if(count_Left%2==0)
//		{
//			Stop();
//			count_Left = count_Left + 1;
//			vTaskSuspend(xHandle_AGV_Action); 
//			if(Direction == Forward )  
//			{
//				vTaskSuspend((xHandle_U2_Navigation));
//			}
//			else if(Direction == Back_Up)
//			{
//				vTaskSuspend((xHandle_U5_Navigation));
//			}
//			vTaskDelay(1000);
//			Left_90Angel();
//			
//			/*
//			假设转弯时间为 Turn_Time
//			*/
//			bsp_DelayMS(Turn_Time);
//			
//			/*
//			假设已经转弯90度
//			*/
//			Stop();
//			vTaskDelay(1000);
//			comClearRxFifo(COM2);
//			Guidance_State = Auto_Wait;
//			vTaskResume(xHandle_AGV_Action);
//			vTaskResume((xHandle_U2_Navigation));
//			//vTaskSuspend(xHandle_Right_90Angel_Check);
//		}
//	
//	}
//	else if(Verify_RFID(&RFID_IN[0],&RFID_90Right[0])) 
//	{
//		/*
//		停车，然后挂起任务，左转，恢复左转90度检测任务
//		*/
//		//if(count_Right%2==0)
//		{
//			Stop();
//			count_Right = count_Right + 1;
//			vTaskSuspend(xHandle_AGV_Action); 
//			if(Direction == Forward )  
//			{
//				vTaskSuspend((xHandle_U2_Navigation));
//			}
//			else if(Direction == Back_Up)
//			{
//				vTaskSuspend((xHandle_U5_Navigation));
//			}
//			 
//			vTaskDelay(1000);
//			Right_90Angel();
//			/*
//			假设转弯时间为 Turn_Time
//			*/
//			bsp_DelayMS(Turn_Time);
//			
//			/*
//			假设已经转弯90度
//			*/
//			Stop();
//			vTaskDelay(1000);
//			comClearRxFifo(COM2);
//			Guidance_State = Auto_Wait;
//			vTaskResume(xHandle_AGV_Action);
//			vTaskResume((xHandle_U2_Navigation));
//			//vTaskSuspend(xHandle_Right_90Angel_Check)
//		}
//		else
//		{
//			Stop();
//			count_Right = count_Right + 1;
//			vTaskDelay(1000);
//			comClearRxFifo(COM2);
//			Guidance_State = Auto_Wait;

//			vTaskResume(xHandle_AGV_Action);
//			vTaskResume((xHandle_U2_Navigation));

//			vTaskSuspend(xHandle_Right_90Angel_Check);
//				
//		}
//		
//	}
	
	
	
	
	
	while(HMI_Command[PLACE_Gone]==0x00)	//剔除不去的目的地
	{
		PLACE_Gone++;
	}
	
	/*
		传递目的地
	*/
	Place = HMI_Command[PLACE_Gone];

	switch(HMI_Command[PLACE_Gone])					
	{			

		//printf("目的地%X\r\n",HMI_Command[PLACE_Gone+3]);
		case 0xFA://起点
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_START[0])) //比较RFID的数据是RFID_START。1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xFB://终点
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_END[0])) //比较RFID的数据是RFID_END。1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xA1: //目的地A1
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A1[0])) //比较RFID的数据是RFID_A1。1表示地点符合，0表示不符合
			{
				
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态
		
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}

		case 0xA2://目的地A2
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A2[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xA3://目的地A3
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A3[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);		
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xA4://目的地A4
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A4[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}			
		case 0xA5://目的地A5
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A5[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB1://目的地B1
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B1[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态		
			}

			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB2://目的地B2
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B2[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB3://目的地B3
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B3[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB4://目的地B4
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B4[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB5://目的地B5
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B5[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态		
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xC1://目的地C1
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C1[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xC2://目的地C2
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C2[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xC3://目的地C3
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C3[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xC4://目的地C4
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C4[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态
					
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}	
		case 0xC5://目的地C5
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C5[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}	
		
		case 0xD1://目的地D1
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D1[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xD2://目的地D2
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D2[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xD3://目的地D3
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D3[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xD4://目的地D4
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D4[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}	
		case 0xD5://目的地D5
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D5[0])) //1表示地点符合，0表示不符合
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //站点停车状态
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}	
		
		default:
		{
			Place_Stop=0;
			break;
		}
	 }

 }



/*******************************************************************************
* Function Name  : void Go_Straight(void) 
* Description    : 控制舵轮直行
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/

void Go_Straight(void) //直走
{
//	GPIO_ResetBits(GPIOD,GPIO_Pin_14); //关闭蜂鸣器
//	GPIO_SetBits(GPIOD,GPIO_Pin_15);// PD15 前进继电器
//	GPIO_ResetBits(GPIOC,GPIO_Pin_8);// PC8	右转继电器
//	GPIO_ResetBits(GPIOC,GPIO_Pin_9);// PC9  左转继电器
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);// PD13 后退继电器
//	//GPIO_SetBits(GPIOD,GPIO_Pin_15);// PD15 前进继电器	
	//1.写 驱动电机伺服器 地址1（功能码 06 地址 02 电机目标速度） 2000 

//	Stop_change = 0;
//	if(Left_change!=Left||Right_change!=Right||Stop_Flag == 1) //防止重复发速度指令
//	{
	
		if(Direction == Forward )  
		{
			Motor1_Speed = (uint16_t)-Speed_Straight;
			Motor2_Speed = (uint16_t)Speed_Straight;
		}
		else if(Direction == Back_Up)
		{
			Motor1_Speed = (uint16_t)Speed_Straight;
			Motor2_Speed =(uint16_t) -Speed_Straight;
		}
	
	
	
		if(MODH_WriteParam_06H(1,2,Motor1_Speed))
		{

		}
		
		if(MODH_WriteParam_06H(2,2,Motor2_Speed))
		{
			
	
		}
//		Left_change = Left;
//		Right_change = Right;
//		Stop_Flag = 0;
//	}
//	

}


 /*******************************************************************************
* Function Name  : void Back_Up(void)
* Description    : 控制舵轮后退
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/



 /*******************************************************************************
* Function Name  : void Turn_Left(void)
* Description    : 控制舵轮左转
* Input          : None	
* Output 		 : None
* Return         : None
左上轮 速度-  往前
右下轮 速度+  往前
*******************************************************************************/
void Turn_Left(uint8_t* data) //左转
{
//	GPIO_ResetBits(GPIOC,GPIO_Pin_8);// PC8	右转继电器
//	GPIO_SetBits(GPIOC,GPIO_Pin_9);// PC9  左转继电器
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);// PD13 后退继电器
//	GPIO_SetBits(GPIOD,GPIO_Pin_15);// PD15 前进继电器
	//MODH_WriteParam_10H(2,0x0c,2,init_buf); 
	uint16_t Speed_Difference_Correction;
	Stop_change = 0;
	
//	if(Left_change!=Left||Right_change!=Right||Stop_Flag == 1) //防止重复发速度指令
//	{
	/*
	*data       1 2 3 4 5 6 7 8
	*(data+1)   9 10 11 12 13 14 15 16
	*/
	//只有当上传的磁导航数据跟上一次的不一样，才再次发信号给电机
	if(new_data1 == *data && new_data2 == *(data+1))
	{
		
	}	
	else
	{
		if((*(data+1)&0x01) == 0x01)  //16
		{
			Speed_Difference_Correction = Speed_Difference*6;
		}
		else if((*(data+1)&0x02) == 0x02) //15
		{
			Speed_Difference_Correction = Speed_Difference*5;
		}
		else if((*(data+1)&0x04) == 0x04) //14
		{
			Speed_Difference_Correction = Speed_Difference*4;
		}
		else if((*(data+1)&0x08) == 0x08) //13
		{
			Speed_Difference_Correction = Speed_Difference*3;
		}
		else if((*(data+1)&0x10)== 0x10) //12
		{
			Speed_Difference_Correction = Speed_Difference*2;
		}
		else if((*(data+1)&0x20) == 0x20) //11
		{
			Speed_Difference_Correction = Speed_Difference*1;
		}
		
		if(Direction == Forward )   
		{
			
			Motor1_Speed = (uint16_t)-Speed_Straight;
			Motor2_Speed = (uint16_t)Speed_Straight-Speed_Difference_Correction;
		}
		else if(Direction == Back_Up)
		{
			
			Motor1_Speed = (uint16_t)Speed_Straight-Speed_Difference_Correction;
			Motor2_Speed = (uint16_t)-Speed_Straight;
		}

		
		if(MODH_WriteParam_06H(1,2,Motor1_Speed))
		{

		}
		
		if(MODH_WriteParam_06H(2,2,Motor2_Speed))
		{
			

		}
		new_data1 = *data;
		new_data2 = *(data+1);
	}
	


//		Left_change = Left;
//		Right_change = Right;
//		Stop_Flag = 0;
//	}
	
}


 /*******************************************************************************
* Function Name  : void Turn_Right(void)
* Description    : 控制舵轮右转
* Input          : None	
* Output 		 : None
* Return         : None
左上轮 速度-  往前
右下轮 速度+  往前
*******************************************************************************/
void Turn_Right(uint8_t* data)//右转
{
//	GPIO_SetBits(GPIOC,GPIO_Pin_8);// PC8	右转继电器
//	GPIO_ResetBits(GPIOC,GPIO_Pin_9);// PC9  左转继电器
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);// PD13 后退继电器
	Stop_change = 0;
	uint16_t Speed_Difference_Correction;	
//	if(Left_change!=Left||Right_change!=Right||Stop_Flag == 1) //防止重复发速度指令
//	{
		/*
	*data       1 2 3 4 5 6 7 8
	*(data+1)   9 10 11 12 13 14 15 16
	*/
	
	if(new_data1 == *data && new_data2 == *(data+1))
	{
		
	}	
	else
	{
		if(((*data)&0x80) == 0x80)  //1
		{
			Speed_Difference_Correction = Speed_Difference*6;
		}
		else if(((*data)&0x40) == 0x40) //2
		{
			Speed_Difference_Correction = Speed_Difference*5;
		}
		else if(((*data)&0x20) == 0x20) //3
		{
			Speed_Difference_Correction = Speed_Difference*4;
		}
		else if(((*data)&0x10) == 0x10) //4
		{
			Speed_Difference_Correction = Speed_Difference*3;
		}
		else if(((*data)&0x08) == 0x08) //5
		{
			Speed_Difference_Correction = Speed_Difference*2;
		}
		else if(((*data)&0x04) == 0x04) //6
		{
			Speed_Difference_Correction = Speed_Difference*1;
		}
		
		
			
		if(Direction == Forward )  
		{
			Motor1_Speed =(uint16_t)-Speed_Straight+Speed_Difference_Correction;
			Motor2_Speed = (uint16_t)Speed_Straight;
		}
		else if(Direction == Back_Up)
		{
			Motor1_Speed = (uint16_t)Speed_Straight;
			Motor2_Speed = (uint16_t)-Speed_Straight+Speed_Difference_Correction;
		}


		if(MODH_WriteParam_06H(1,2,Motor1_Speed))
		{

		}
		
		if(MODH_WriteParam_06H(2,2,Motor2_Speed))
		{
			

		}
	
		new_data1 = *data;
		new_data2 = *(data+1);

	}
	
//		Left_change = Left;
//		Right_change = Right;
//		Stop_Flag = 0;
//	}

}


 /*******************************************************************************
* Function Name  : void Stop(void)
* Description    : 控制舵轮停止
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void Stop(void)//停车
{
//	GPIO_SetBits(GPIOD,GPIO_Pin_14); //打开蜂鸣器
//	GPIO_ResetBits(GPIOC,GPIO_Pin_8);//置0 PC8	右转继电器断开
//	GPIO_ResetBits(GPIOC,GPIO_Pin_9);//置0 PC9  左转继电器断开
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);// PD13 后退继电器
//	GPIO_ResetBits(GPIOD,GPIO_Pin_15);//置0 PD15 前进继电器断开
	
	//MODH_WriteParam_06H(1,2,0);		//驱动伺服器速度模式，目标速度写0
	Stop_Flag = 1;
	//驱动电机
//	if(Left_change!=Left||Right_change!=Right||Stop_change !=Stop_Flag) //防止重复发速度指令
//	{

		if(MODH_WriteParam_06H(1,2,0))//驱动伺服器速度模式，目标速度写0，即可停车
		{

		}
//		
		if(MODH_WriteParam_06H(2,2,0))//驱动伺服器速度模式，目标速度写0，即可停车
		{

		}
//		Left_change = Left;
//		Right_change = Right;
//		Stop_change = Stop_Flag;
//		
//	}
	//bsp_DelayMS(1000);
}



 /*******************************************************************************
* Function Name  : void Alarm(void)
* Description    : AGV报警
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void Alarm(void)//报警
{
	//GPIO_SetBits(GPIOD,GPIO_Pin_14); //打开蜂鸣器
}


 /*******************************************************************************
* Function Name  : void HMI_END(void)
* Description    : 发送结束符给HMI
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/

void HMI_END(void)
{

	bsp_DelayMS(100);
	USART_SendData(USART6,0xFF);
	bsp_DelayMS(50);
	USART_SendData(USART6,0xFF);
	bsp_DelayMS(50);
	USART_SendData(USART6,0xFF);
	bsp_DelayMS(50);
}



 /*******************************************************************************
* Function Name  : void RFID_Check(void)
* Description    : RFID初始化检测函数，判断RFID读卡器是否在线
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void RFID_Check(void)
{
	

	/*
	U3_send_buf 给RFID发送的数据，工作模式设置为自动读块数据，块号为1，数据上传模式为自动上传模式
	U3_Respond_buf 为RFID读卡器设置成功后返回的数据
	*/
	uint8_t U3_send_buf[8]    = {0x3,0x8,0xC1,0x20,0x3,0x1,0x0,0x17}; 
	uint8_t U3_Respond_buf[8] = {0x3,0x8,0xC1,0x20,0x0,0x0,0x0,0x15};
	
	uint8_t U3_si;
	uint8_t U3_check_read;
	uint8_t RFID_Correct = 0;
	uint8_t U3_check_status = 0;
	uint8_t U3_First_Check = 1;
////设置工作模式  自动读取块数据   块1  自动上传数据
	//03 08 C1 20 03 01 00 17  	Send
	//03 08 C1 20 00 00 00 15	Respond
	for(U3_si=0;U3_si<8;U3_si++)
	{
		USART_SendData(USART3,U3_send_buf[U3_si]);
		bsp_DelayMS(10);
	}

	bsp_DelayMS(500);

	while(RFID_Correct != 1)
	{
		if(U3_First_Check)
		{
			printf("调试窗口.显示区.txt=\"RFID读卡器初始化中\"");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);
		}
		U3_First_Check = 0;

		if (comGetChar(COM3, &U3_check_read))
		{

			switch (U3_check_status)
			{

				case 0:
				{
					if(U3_check_read == U3_Respond_buf[U3_check_status])
					{
						U3_check_status = 1;   


					}
					break;
				}                  

				case 1:
				{
					if(U3_check_read == U3_Respond_buf[U3_check_status])
					{
						U3_check_status = 2;   


					}
					break;
				}  
				case 2:
				{
					if(U3_check_read == U3_Respond_buf[U3_check_status])
					{
						U3_check_status = 3;   


					}
					break;
				}  			
				case 3:
				{
					if(U3_check_read == U3_Respond_buf[U3_check_status])
					{
						U3_check_status = 4;   


					}
					break;
				}
				case 4:
				{
					if(U3_check_read == U3_Respond_buf[U3_check_status])
					{
						U3_check_status = 5;   


					}
					break;
				}
				case 5:
				{
					if(U3_check_read == U3_Respond_buf[U3_check_status])
					{
						U3_check_status = 6;   


					}
					break;
				}		
				case 6:
				{
					if(U3_check_read == U3_Respond_buf[U3_check_status])
					{
						U3_check_status = 7;   


					}
					break;
				}	
				case 7:
				{
					if(U3_check_read == U3_Respond_buf[U3_check_status])
					{
	 
						U3_check_status = 0;

						RFID_Correct = 1;
					}
					else
					{   
						U3_check_status = 0;
						RFID_Correct = 0;
					}
					break;
				}	
				
				default:
					break;
			}
		}

	}

	bsp_DelayMS(100);
	printf("调试窗口.显示区.txt=\"RFID读卡器初始化成功\"");
	bsp_DelayMS(300);
	HMI_END();
	bsp_DelayMS(1000);
		
}

// /*******************************************************************************
//* Function Name  : void Attitude_Check(void)
//* Description    : 超声波传感器初始化检测函数，判断超声波传感器是否在线
//* Input          : None	
//* Output 		 : None
//* Return         : None
//*******************************************************************************/

//void Attitude_Check(void) //姿态传感器检测
//{
//	uint8_t U5_Attitude_Checked = 0;
//	while(Attitude_Correct!=1)
//	{
//		U5_State( ); //状态机
//		
//		//姿态传感器检测
//		if(!U5_Attitude_Checked)
//		{
//			printf("调试窗口.显示区.txt=\"姿态传感器偏航角归零中\"");
//			bsp_DelayMS(300);
//			HMI_END();
//			bsp_DelayMS(100);

//		}
//		U5_Attitude_Checked =1; //为了只往HMI发一次数据，设标志位
//	}
//	printf("调试窗口.显示区.txt=\"姿态传感器偏航角归零成功\"");
//	bsp_DelayMS(300);
//	HMI_END();
//	bsp_DelayMS(1000);
//	

//}
 /*******************************************************************************
* Function Name  : void U2_Navigation_Check(void)
* Description    : 磁导航传感器初始化检测函数，判断磁导航传感器是否在线
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void U2_Navigation_Check(void) //磁导航检测
{


	while(U2_Navigation_Correct!=1)
	{
		U2_State(); //状态机

		if(!U2_Navigation_Checked)
		{
			printf("调试窗口.显示区.txt=\"串口2 磁导航传感器初始化中\"");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);

		}
		U2_Navigation_Checked =1;
	}
	printf("调试窗口.显示区.txt=\"串口2 磁导航传感器初始化成功\"");
	bsp_DelayMS(300);
	HMI_END();
	bsp_DelayMS(1000);
	U2_ucCount = 0;

}


 /*******************************************************************************
* Function Name  : void U5_Navigation_Check(void)
* Description    : 磁导航传感器初始化检测函数，判断磁导航传感器是否在线
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void U5_Navigation_Check(void) //磁导航检测
{

	//uint8_t U2_Navigation_Checked = 0;
	while(U5_Navigation_Correct!=1)
	{
		U5_State(); //状态机

		if(!U5_Navigation_Checked)
		{
			printf("调试窗口.显示区.txt=\"串口5 磁导航传感器初始化中\"");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);

		}
		U5_Navigation_Checked =1;
	}
	printf("调试窗口.显示区.txt=\"串口5 磁导航传感器初始化成功\"");
	bsp_DelayMS(300);
	HMI_END();
	bsp_DelayMS(1000);
	U5_ucCount = 0;

}



 /*******************************************************************************
* Function Name  : void RFID_TO_HMI(void)
* Description    : 读卡判断为目的地之后，发送当前停车点给HMI触摸屏
* Input          : uint8_t place	
* Output 		 : None
* Return         : None
*******************************************************************************/
void RFID_TO_HMI(uint8_t place)
{
	
	Stop();

	Place_Stop=1;
	Send_First=1;
	PLACE_Gone++;
	printf("%X.val=1",place);
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(100);	



	vTaskSuspend(xHandle_U2_Navigation );


}

 /*******************************************************************************
* Function Name  : void AGV_To_Network(void)
* Description    : 通过网络发送AGV状态函数
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_To_Network(uint8_t *_Work_Mode,uint8_t *_Reserved)    //通过网络发送AGV状态函数
{

	int i;
	
	uint8_t testbuf[1] = {0x0a};
	/*
	发送包头
	*/
//	Network_Reserved_BUF[0] = Network_Head_BUF[0];
//	Network_Reserved_BUF[1] = Network_Head_BUF[1];
	for(i=0;i<2;i++)
	{
		Network_AGV_BUF[i] = Network_Head_BUF[i];
	}
	
	
	
	/*
	AGV设备标识
	*/	
	for(i=0;i<3;i++)
	{
		Network_AGV_BUF[i+2] = Network_AGVID_BUF[i];
	}


	
	
	/*
	AGV工作模式
	*/
	Network_AGV_BUF[5] = *_Work_Mode;

	
	
	/*
	 AGV状态 + 前往地点
	*/	
	Network_AGV_BUF[6] = State;
	Network_AGV_BUF[7] = Place;
	
	/*
	 8个预留 字节
	*/	
	
	
	//Network_Reserved_Send(_Reserved);
	
	
	
	
	/*
	发送包尾
	*/	
	Network_AGV_BUF[16] = Network_End_BUF[0];
	
	comSendBuf(COM4, (uint8_t *)Network_AGV_BUF, sizeof(Network_AGV_BUF));
	comSendBuf(COM4, (uint8_t *)testbuf, sizeof(testbuf));
	
}


 /*******************************************************************************
* Function Name  : void Network_Head_Send(void)
* Description    : 发送通讯协议包头
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void Network_Head_Send(void) 
{
	
	comSendBuf(COM4, (uint8_t *)Network_Head_BUF, sizeof(Network_Head_BUF));

}
 /*******************************************************************************
* Function Name  : void Network_AGVID_Send(void)
* Description    : 发送 AGV ID 每台AGV有独立的ID，需要预设
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void Network_AGVID_Send(void) 
{
	
	comSendBuf(COM4, (uint8_t *)Network_AGVID_BUF, sizeof(Network_AGVID_BUF));

}


 /*******************************************************************************
* Function Name  : void Network_AGVState_Send(void)
* Description    : 发送 AGV 当前状态
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void Network_AGVState_Send(uint8_t *_State,uint8_t *_Place) 
{
	uint8_t *None = 0;
	comSendBuf(COM4, (uint8_t *)_State, Network_State_LEN);
	/*
		_State == 0xC1表示待机
		目前待机无法判断地点，而且正常情况下，只有当回到起点的时候才处于待机情况
	*/
	if(*_State == 0xC1)
	{
		comSendBuf(COM4, (uint8_t *)None, Network_Place_LEN);
	}
	else
	{
		comSendBuf(COM4, (uint8_t *)_Place, Network_Place_LEN);
	}


}
 /*******************************************************************************
* Function Name  : void Network_Reserved_Send(void)
* Description    : 预留了8个字节空间，做后期扩展
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void Network_Reserved_Send(uint8_t *Reserved) 
{
	comSendBuf(COM4, (uint8_t *)Reserved, Network_Reserved_LEN);
}


 /*******************************************************************************
* Function Name  : void Network_End_Send(void)
* Description    : 发送协议包尾
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void Network_End_Send(void) 
{
	//Network_Head_BUF
	comSendBuf(COM4, (uint8_t *)Network_End_BUF, sizeof(Network_End_BUF));

}



 /*******************************************************************************
* Function Name  : void Task_Resume(void)
* Description    : 恢复任务函数
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void Task_Resume(void)
{
	vTaskResume(xHandle_U2_Navigation); 
	vTaskResume(xHandle_U3_RFID);
	//vTaskResume(xHandle_U5_Navigation); 
	vTaskResume(xHandle_AGV_Action); 

}

 /*******************************************************************************
* Function Name  : void Running_Green_Light(void)
* Description    : 正常运行时绿灯亮
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/

void Running_Green_Light(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_15); 	//G
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);	//Y			
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);	//R	
	//printf("绿灯亮\r\n");

}

 /*******************************************************************************
* Function Name  : void Waiting_Yellow_Light(void)
* Description    : 站点停靠时黄灯亮
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/

void Waiting_Yellow_Light(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_15); 	//G
	GPIO_SetBits(GPIOC,GPIO_Pin_9);	 	//Y			
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);	//R	
	//printf("黄灯亮\r\n");

}

 /*******************************************************************************
* Function Name  : void Alarm_Red_Light(void)
* Description    : 站点停靠时黄灯亮
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/

void Alarm_Red_Light(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_15); 	//G
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);	//Y			
	GPIO_SetBits(GPIOC,GPIO_Pin_8);		//R	
	//printf("红灯亮\r\n");

}

 /*******************************************************************************
* Function Name  : void AGV_Have_Barrier(void)
* Description    :  AGV前方有障碍物，AGV停车，并且马上挂起 vTaskSuspend(xHandle_AGV_Action);
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_Have_Barrier(void)
{
	Stop();
	/*
	 挂起AGV工作
	*/
	vTaskSuspend(xHandle_AGV_Action);
	vTaskSuspend(xHandle_U2_Navigation);
	vTaskSuspend(xHandle_U5_Navigation);
	/*
	  故障时候红色灯亮
	*/
	Alarm_Red_Light();	
	
	State = Barrier_ERR  ; //障碍物状态
	printf("行驶界面.障碍物.val=1");
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(1000);
	
	
}
 /*******************************************************************************
* Function Name  : void AGV_None_Barrier(void)
* Description    : AGV前方没有障碍物，恢复运行vTaskResume(xHandle_AGV_Action);
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_None_Barrier(void)
{	
	State = Moving  ; 
	Barrier = 0;
	Guidance_State = Auto_Wait;
	printf("行驶界面.障碍物.val=0");
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(1000);
	vTaskResume(xHandle_U2_Navigation);
	vTaskResume(xHandle_U5_Navigation);
	vTaskResume(xHandle_AGV_Action);

}



 /*******************************************************************************
* Function Name  : void AGV_StopBManual(void)
* Description    : 手动停止
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_StopBManual(void)
{
	Stop();
	/*
	 挂起AGV工作
	*/
	vTaskSuspend(xHandle_AGV_Action);
	vTaskSuspend(xHandle_U2_Navigation);
	vTaskSuspend(xHandle_U5_Navigation);
	
	State = Manual_Stop  ; //手动停止
	/*
	  故障时候黄色灯亮
	*/
	Waiting_Yellow_Light();	

}
 /*******************************************************************************
* Function Name  : void AGV_RunBManual(void)
* Description    : 手动恢复
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_RunBManual(void)
{	
	State = Moving  ; 
	//Barrier = 0;
	Guidance_State = Auto_Wait;

	vTaskResume(xHandle_U2_Navigation);
	vTaskResume(xHandle_U5_Navigation);
	vTaskResume(xHandle_AGV_Action);
}

 /*******************************************************************************
* Function Name  : void U2_Left_90Angel_Check(void)
* Description    : 判断AGV是否已经左转90度   根据U2磁导航传感器
* Input          : 
* Output 		 : None
* Return         : None

	
		*data 		01~08
		*(data+1)	09~16
	
		当01~05探头被触发，右转
		
		当12~16探头被触发，左转
	
*******************************************************************************/
void U2_Left_90Angel_Check(void)
{	
	/*
		判断方法：
		1. 5 6 7 8 9 10 11 12探头任都没感应到信号，认为是开始拐弯
		//2. 5 6 7 8 9 10 11 12探头任一一个被触发，认为是结束拐弯
		2. 7 8 9 10 探头任一一个被触发，认为是结束拐弯
	*/

	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //返回数据的包头 5个字节    十六路磁导航
//	uint8_t Navigation_9_10 ; //9 10号探头
//	uint8_t Navigation_7_8 ; //7 8号探头
	uint8_t Navigation_10_11_12_13; //10 11 12 13
	uint8_t ALL_1_8;
	uint8_t	ALL_9_16;
	uint8_t Left_90Angel_Check;
	if (comGetChar(COM2, &U2_read))
	{
		switch (U2_ucStatus)
		{
			/* 状态0保证接收到0x01 */
			case 0:
			{
				if(U2_read == CDH_HEAD[U2_ucStatus])
				{
					U2_ucStatus = 1;   

				}
				break;
			}                  

			case 1:
			{
				if(U2_read == CDH_HEAD[U2_ucStatus])
				{
					U2_ucStatus = 2;
					U2_Navigation_Correct =1;
				}
				else
				{
					U2_ucStatus = 0;

				}
				break;
			}  

			case 2:
			{
				if(U2_read!=0x55) ///这里有个BUG,第一笔数据为0X55
				{
					U2_buf[U2_ucCount] = U2_read;              
					 /* 接收够2个数据 */
					if(U2_ucCount == 2-1)
					{	
						//printf("有数据进来\r\n");
						U2_ucStatus = 0;
						U2_ucCount=0;
						UART2_Done = 1;
						Left_90Angel_Check =1;
						//Navigation_9_10 = (U2_buf[1] &0xC0); //9 10 
						//Navigation_7_8 = (U2_buf[0] &0x03); //7 8
						Navigation_10_11_12_13=(U2_buf[1] &0xFF);
						ALL_1_8 =  (U2_buf[0] &0xFF);
						ALL_9_16 = (U2_buf[1] &0xFF);
							
					}
					else
					{
						U2_ucCount++;
					}
				}
				else
				{
					U2_ucStatus = 0;
					U2_ucCount=0;
				}
				break;	
			}
		    default:
				break;
		}
	}
	//只有当 Left_90Angel_Check = 1 才执行下面代码
	//当磁导航上传的数据包头包尾都正确，Left_90Angel_Check才等于1
	if(Left_90Angel_Check ==1 )
	{
		//Left_90Angel_Check = 0;
		switch (State_16)
		{
			case 0:
			{
				if((ALL_1_8 ==0) &&(ALL_9_16 ==0)) //16号为0
				{
					State_16 = 1;   
				}
						
				break;
			}
			
			case 1:
			{
				//if((Navigation_7_8 == 0x03)&& (Navigation_9_10 == 0xC0)) // 7 8 9 10 同时检测到磁条
				if(Navigation_10_11_12_13) //10 11 12 13 同时检测到磁条
				{
					State_16 = 0;   
					/*
						已经右转90度
					*/
					Stop();
					Left_90Angel_Check = 0;
					vTaskDelay(1000);
					comClearRxFifo(COM2);
					Guidance_State = Auto_Wait;

					vTaskResume(xHandle_AGV_Action);
					vTaskResume((xHandle_U2_Navigation));

					vTaskSuspend(xHandle_Right_90Angel_Check);
				}													
				break;
			}
		}
		
		
	}
	
}
 /*******************************************************************************
* Function Name  : void Right_90Angel_Check(void)
* Description    : 判断AGV是否已经右转90度 根据U2 磁导航
* Input          : 
* Output 		 : None
* Return         : None




*******************************************************************************/
void U2_Right_90Angel_Check(void)
{	
	/*
		判断方法：
		1. 所有探头任都没感应到信号，认为是开始拐弯
		2. 5 6 7 8 9 10 11 12探头任一一个被触发，认为是结束拐弯
	*/

	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //返回数据的包头 5个字节    十六路磁导航
//	uint8_t Navigation_9_10 ; //9 10号探头
//	uint8_t Navigation_7_8 ; //7 8号探头
	//uint8_t Navigation_4567;
	uint8_t Navigation_5678;
	uint8_t ALL_1_8;
	uint8_t	ALL_9_16;
	uint8_t Right_90Angel_Check;
	if (comGetChar(COM2, &U2_read))
	{
		switch (U2_ucStatus)
		{
			/* 状态0保证接收到0x01 */
			case 0:
			{
				if(U2_read == CDH_HEAD[U2_ucStatus])
				{
					U2_ucStatus = 1;   
					//printf("0x55\r\n");
					//ucCount++;
				}
				break;
			}                  

			case 1:
			{
				if(U2_read == CDH_HEAD[U2_ucStatus])
				{
					U2_ucStatus = 2;
					//printf("0x54\r\n");
					//ucCount++;
					U2_Navigation_Correct =1;
				}
				else
				{
					U2_ucStatus = 0;
					//ucCount  = 0;
				}
				break;
			}  

			case 2:
			{

				/*
					data1 data2 CRC  CRC =  data1 + data2
				*/
				U2_buf[U2_ucCount] = U2_read; 
				if(U2_ucCount==2)
				{
					if(U2_buf[2] == (U2_buf[0]+U2_buf[1]))
					{
//						U2_ucStatus = 0;
//						U2_ucCount=0;
						UART2_Done = 1;	
						Right_90Angel_Check =1;
						//Navigation_9_10 = (U2_buf[1] &0xC0); //9 10 
						//Navigation_7_8 = (U2_buf[0] &0x03); //7 8
						//Navigation_4567 = (U2_buf[0] &0x1E); //4567
						Navigation_5678 = (U2_buf[0] &0x0F); //5678
						ALL_1_8 =  (U2_buf[0] &0xFF);
						ALL_9_16 = (U2_buf[1] &0xFF);
					}
					U2_ucStatus = 0;
					U2_ucCount=0;		
				}
				else
				{
					U2_ucCount++;
				}
				break;	
			}
		   default:
				break;
		}
	}
	//只有当 Right_90Angel_Check = 1 才执行下面代码
	//当磁导航上传的数据包头包尾都正确，Right_90Angel_Check才等于1
	if(Right_90Angel_Check ==1 )
	{
		Right_90Angel_Check = 0;
		switch (State_16)
		{
			case 0:
			{
				if((ALL_1_8 ==0) &&(ALL_9_16 ==0)) // 1.都离开磁条
				{
					State_16 = 1;   
				}
						
				break;
			}
			
			case 1:
			{
				//if((Navigation_7_8 == 0x03)&& (Navigation_9_10 == 0xC0)) // 7 8 9 10 同时检测到磁条
				if(Navigation_5678 == 0x0F) // 4 5 6 7 同时检测到磁条
				{
					State_16 = 0;   
					/*
						已经右转90度
					*/
					Stop();
					vTaskDelay(1000);
					comClearRxFifo(COM2);
					Guidance_State = Auto_Wait;

					vTaskResume(xHandle_AGV_Action);
					vTaskResume((xHandle_U2_Navigation));

					vTaskSuspend(xHandle_Right_90Angel_Check);
				}													
				break;
			}
		}
	}
	
}
 /*******************************************************************************
* Function Name  : void U5_Left_90Angel_Check(void)
* Description    : 判断AGV是否已经左转90度   根据U5磁导航传感器
* Input          : 
* Output 		 : None
* Return         : None

	
		*data 		01~08
		*(data+1)	09~16
	
		当01~05探头被触发，右转
		
		当12~16探头被触发，左转
	
*******************************************************************************/
void U5_Left_90Angel_Check(void)
{	
//	/*
//		判断方法：
//		磁条为十字形，当从一条磁条离开，再转移到相邻的磁条上，则近似为90度
//		当01号探头从0到1再到0，判断为离开了当前磁条
//		当07号探头为1，则判断为已经转移到新的磁条上
//	*/

	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //返回数据的包头 5个字节    十六路磁导航
	uint8_t Navigation_9_10 ; //16号探头
	uint8_t Navigation_7_8 ; //10号探头
	uint8_t ALL_1_8;
	uint8_t	ALL_9_16;
	uint8_t Left_90Angel_Check;
	if (comGetChar(COM5, &U5_read))
	{
		switch (U5_ucStatus)
		{
			/* 状态0保证接收到0x01 */
			case 0:
			{
				if(U5_read == CDH_HEAD[U5_ucStatus])
				{
					U5_ucStatus = 1;   

				}
				break;
			}                  

			case 1:
			{
				if(U5_read == CDH_HEAD[U5_ucStatus])
				{
					U5_ucStatus = 2;
					U5_Navigation_Correct =1;
				}
				else
				{
					U5_ucStatus = 0;

				}
				break;
			}  

			case 2:
			{
				if(U5_read!=0x55) ///这里有个BUG,第一笔数据为0X55
				{
					U5_buf[U5_ucCount] = U5_read;              
					 /* 接收够2个数据 */
					if(U5_ucCount == 2-1)
					{	
						U5_ucStatus = 0;
						U5_ucCount=0;
						UART5_Done = 1;
						Left_90Angel_Check =1;
						Navigation_9_10 = (U2_buf[1] &0xC0); //9 10 
						Navigation_7_8 = (U2_buf[0] &0x03); //7 8
						ALL_1_8 =  (U2_buf[0] &0xFF);
						ALL_9_16 = (U2_buf[1] &0xFF);
							
					}
					else
					{
						U5_ucCount++;
					}
				}
				else
				{
					U5_ucStatus = 0;
					U5_ucCount=0;
				}
				break;	
			}
		    default:
				break;
		}
	}
	//只有当 Left_90Angel_Check = 1 才执行下面代码
	//当磁导航上传的数据包头包尾都正确，Left_90Angel_Check才等于1
	if(Left_90Angel_Check ==1 )
	{
		Left_90Angel_Check = 0;
		switch (State_16)
		{
			case 0:
			{
				if((ALL_1_8 ==0) &&(ALL_9_16 ==0)) //16号为0
				{
					State_16 = 1;   
				}
						
				break;
			}
			
			case 1:
			{
				if((Navigation_7_8 == 0x03)&& (Navigation_9_10 == 0xC0)) // 7 8 9 10 同时检测到磁条
				{
					State_16 = 0;   
					/*
						已经右转90度
					*/
					Stop();
					vTaskDelay(1000);
					comClearRxFifo(COM5);
					Guidance_State = Auto_Wait;

					vTaskResume(xHandle_AGV_Action);
					vTaskResume((xHandle_U5_Navigation));

					vTaskSuspend(xHandle_Right_90Angel_Check);
				}													
				break;
			}
		}
		
		
	}
	
	
	
}
 /*******************************************************************************
* Function Name  : void U5_Right_90Angel_Check(void)
* Description    : 判断AGV是否已经右转90度 根据U5 磁导航
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void U5_Right_90Angel_Check(void)
{	
	/*
		判断方法：
		磁条为十字形，当从一条磁条离开，再转移到相邻的磁条上，则近似为90度
		当16号探头从0到1再到0，判断为离开了当前磁条
		当10号探头为1，则判断为已经转移到新的磁条上
	*/

	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //返回数据的包头 5个字节    十六路磁导航
	uint8_t Navigation_9_10 ; //16号探头
	uint8_t Navigation_7_8 ; //10号探头
	uint8_t ALL_1_8;
	uint8_t	ALL_9_16;
	uint8_t Right_90Angel_Check;
	if (comGetChar(COM5, &U5_read))
	{
		switch (U5_ucStatus)
		{
			/* 状态0保证接收到0x01 */
			case 0:
			{
				if(U5_read == CDH_HEAD[U5_ucStatus])
				{
					U5_ucStatus = 1;   

				}
				break;
			}                  

			case 1:
			{
				if(U5_read == CDH_HEAD[U5_ucStatus])
				{
					U5_ucStatus = 2;

					U5_Navigation_Correct =1;
				}
				else
				{
					U5_ucStatus = 0;
				
				}
				break;
			}  

			case 2:
			{
				if(U5_read!=0x55) ///这里有个BUG,第一笔数据为0X55
				{
					U5_buf[U5_ucCount] = U5_read;              
					 /* 接收够2个数据 */
					if(U5_ucCount == 2-1)
					{	

						U5_ucStatus = 0;
						U5_ucCount=0;
						UART5_Done = 1;
						Right_90Angel_Check =1;
						Navigation_9_10 = (U2_buf[1] &0xC0); //9 10 
						Navigation_7_8 = (U2_buf[0] &0x03); //7 8
						ALL_1_8 =  (U2_buf[0] &0xFF);
						ALL_9_16 = (U2_buf[1] &0xFF);
							
					}
					else
					{
						U5_ucCount++;
					}
				}
				else
				{
					U5_ucStatus = 0;
					U5_ucCount=0;
				}
				break;	
			}
		   default:
				break;
		}
	}
	//只有当 Right_90Angel_Check = 1 才执行下面代码
	//当磁导航上传的数据包头包尾都正确，Right_90Angel_Check才等于1
	if(Right_90Angel_Check ==1 )
	{
		Right_90Angel_Check = 0;
		switch (State_16)
		{
			case 0:
			{
				if((ALL_1_8 ==0) && (ALL_9_16 ==0)) // 1.都离开磁条
				{
					State_16 = 1;   
				}
						
				break;
			}
			
			case 1:
			{
				if((Navigation_7_8 == 0x03)&& (Navigation_9_10 == 0xC0)) // 7 8 9 10 同时检测到磁条
				{
					State_16 = 0;   
					/*
						已经右转90度
					*/
					Stop();
					vTaskDelay(1000);
					comClearRxFifo(COM5);
					Guidance_State = Auto_Wait;

					vTaskResume(xHandle_AGV_Action);
					vTaskResume((xHandle_U5_Navigation));

					vTaskSuspend(xHandle_Right_90Angel_Check);
				}													
				break;
			}
		}
	}
	
}
 /*******************************************************************************
* Function Name  : void Left_90Angel(void)
* Description    : 低速左转90度
* Input          : 
* Output 		 : None
* Return         : None
左上轮 速度-  往前
右下轮 速度+  往前
*******************************************************************************/
void Left_90Angel(void)
{
	

	//if(MODH_WriteParam_06H(1,2,(uint16_t)(-Speed_90Angle+Speed_90Angle_Difference)))
	//while(MODH_WriteParam_06H(1,2,(uint16_t)-(Speed_90Angle))!=1)
	//if(MODH_WriteParam_06H(1,2,0))
	if(MODH_WriteParam_06H(1,2,(uint16_t)-(Speed_90Angle)))
	{
			
	
	}
		
	//while(MODH_WriteParam_06H(2,2,(uint16_t)-Speed_90Angle)!=1)
	if(MODH_WriteParam_06H(2,2,(uint16_t)-Speed_90Angle))
	{
			
	
	}

}

 /*******************************************************************************
* Function Name  : void Right_90Angel(void)
* Description    : 低速右转90度
* Input          : 
* Output 		 : None
* Return         : None
左上轮 速度-  往前
右下轮 速度+  往前
*******************************************************************************/
void Right_90Angel(void)
{
//	if(MODH_WriteParam_06H(1,2,(uint16_t)Speed_90Angle)) 
//	{
//			
//	
//	}
//	if(MODH_WriteParam_06H(2,2,(uint16_t)Speed_90Angle))
//	{

//	
//	}
	taskENTER_CRITICAL();
	if(MODH_WriteParam_06H(1,2,(uint16_t)Speed_90Angle)) 
	{
			
	
	}
	if(MODH_WriteParam_06H(2,2,(uint16_t)Speed_90Angle))
	{

	
	}
	taskEXIT_CRITICAL();


}

 /*******************************************************************************
* Function Name  : void go_straight(void)
* Description    : 直走
* Input          : 
* Output 		 : None
* Return         : None
左上轮 速度-  往前
右下轮 速度+  往前
go_back();
*******************************************************************************/
void go_straight(void)
{
	//while(MODH_WriteParam_06H(1,2,(uint16_t)-Speed_Straight_GO)!=1) 
	if(MODH_WriteParam_06H(1,2,(uint16_t)-Speed_Straight_GO)) 	
	{
			
	
	}
		
	if(MODH_WriteParam_06H(2,2,(uint16_t)Speed_Straight_GO))
	//if(MODH_WriteParam_06H(2,2,0))
	{

	
	}
	

}

 /*******************************************************************************
* Function Name  : void go_back(void)
* Description    : 后退
* Input          : 
* Output 		 : None
* Return         : None
左上轮 速度-  往前
右下轮 速度+  往前

*******************************************************************************/
void go_back(void)
{
	if(MODH_WriteParam_06H(1,2,(uint16_t)Speed_Straight_GO)) 
	{
			
	
	}
		
	if(MODH_WriteParam_06H(2,2,(uint16_t)-Speed_Straight_GO))
	//if(MODH_WriteParam_06H(2,2,0))
	{

	
	}
	

}

void TEST(void)
{
	vTaskDelay(1000);
	comClearRxFifo(COM2);
	Guidance_State = Auto_Wait;

	vTaskResume(xHandle_AGV_Action);
	vTaskResume((xHandle_U2_Navigation));

	vTaskSuspend(xHandle_Right_90Angel_Check);

}

/*

TBD:暂定用于判断转弯是否超时
若10秒内还没检测到磁条，就判定转弯超时
*/
void Time_Out(void)
{
	//int32_t turn_time;
	//turn_time = bsp_GetRunTime();	/* 记录命令发送的时刻 */
//	if (bsp_CheckRunTime(turn_time) > TIMEOUT_Turn)	
//	{
//	
//	}

}


 /*******************************************************************************
* Function Name  : void Path_Choice(void)
* Description    : 路径选择
* Input          : 目的地 uint8_t Place;
* Output 		 : 直行、左转、右转信号
* Return         : 分岔路路径选择标志位
当读取到分叉口2的时候，需要判断目前需要前往的目的地位于哪个方向

直行：目的地是C段 直行段
左转：目的地是B段 左转段
右转：目的地是D段 右转段

把地点分为段，相当于集合
通过遍历数组，判断目的地属于哪个集合
uint8_t Straight_Sets[10]   = {0xC1,0xC2,0xC3,0xC4,0xC5};
uint8_t Left_Turn_Sets[10]  = {0xB1,0xB2,0xB3,0xB4,0xB5};
uint8_t Right_Turn_Sets[10] = {0xD1,0xD2,0xD3,0xD4,0xD5};
uint8_t Back_Turn_Sets[10] = {0xA1,0xA2,0xA3,0xA4,0xA5};

分叉口选择标志位

Straight_Flag = 1;
Left_Flag = 2;
Right_Flag = 3;

*******************************************************************************/
int Path_Choice(uint8_t Order_Place)
{
	int i;
	int state=0;
	int Path_Flag = 0;
	/*
	判断是否属于C段 直行段
	*/
	switch(state)
	{
		case 0:
		{
			for(i=0;i<Straight_Len;i++)
			{
				if(Order_Place != Straight_Sets[i])
				{
					state = 1;
				}
				else
				{
					Path_Flag = Straight_Flag;
					state = 0;

					break;
				}
			}
		}
		case 1:
		{
			for(i=0;i<Left_Len;i++)
			{
				if(Order_Place != Left_Turn_Sets[i])
				{
					//Left_Flag = 0;
					state = 2;
				}
				else
				{
					Path_Flag = Left_Flag;
					state = 0;
					break;	
				}
			}
		}
		case 2:
		{
			for(i=0;i<Right_Len;i++)
			{
				if(Order_Place != Right_Turn_Sets[i])
				{
					//Right_Flag = 0;
					state = 0;
				}
				else
				{
					Path_Flag =  Right_Flag;
					state = 0;
					break;
				}
			 }
	    }
		case 3:
		{
			for(i=0;i<Back_Len;i++)
			{
				if(Order_Place != Back_Turn_Sets[i])
				{
					//Right_Flag = 0;
					state = 0;
				}
				else
				{
					Path_Flag =  Back_Flag;
					state = 0;
					break;
				}
			 }
	    }
	}
	return Path_Flag;
}

/*******************************************************************************
* Function Name  : void Fork_Left(void)
* Description    : 分岔路左转
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void Fork_Left(void)
{
	//int time_count = Turn_Time;
	//Stop();
	vTaskSuspend(xHandle_AGV_Action); 
	Stop();
	if(Direction == Forward )  
	{
		vTaskSuspend((xHandle_U2_Navigation));
		comClearRxFifo(COM2);
	}
	else if(Direction == Back_Up)
	{
		vTaskSuspend((xHandle_U5_Navigation));
		comClearRxFifo(COM5);
	}
	vTaskDelay(2000);
	Left_90Angel();
	vTaskResume(xHandle_Left_90Angel_Check); 
//	while(1)
//	{
//		//bsp_DelayMS(1000);	
//		//printf("进入循环");
//		Read_Speed();
//		Send_Speed();
//	}
//	/*
//	假设转弯时间为 Turn_Time
//	*/
//	while(time_count)
//	{
//		bsp_DelayMS(1000);
//		time_count = time_count - 1;
//	}

//	
//	/*
//	假设已经转弯90度
//	*/
//	Stop();
//	
//	vTaskDelay(1000);
//	comClearRxFifo(COM2);
//	Guidance_State = Auto_Wait;
//	vTaskResume(xHandle_AGV_Action);
//	vTaskResume((xHandle_U2_Navigation));
			
	//vTaskResume(xHandle_Left_90Angel_Check); 

}
/*******************************************************************************
* Function Name  : void Fork_Right(void)
* Description    : 分岔路右转
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void Fork_Right(void)
{
	//int time_count = Turn_Time;
	//Stop();
	vTaskSuspend(xHandle_AGV_Action); 
	Stop();
	if(Direction == Forward )  
	{
		vTaskSuspend((xHandle_U2_Navigation));
		comClearRxFifo(COM2);
	}
	else if(Direction == Back_Up)
	{
		vTaskSuspend((xHandle_U5_Navigation));
		comClearRxFifo(COM5);
	}
	 
	//vTaskDelay(1000);
	Right_90Angel();
	vTaskDelay(1000);
	vTaskResume(xHandle_Right_90Angel_Check);
//	
//	/*
//	假设已经转弯90度
//	*/
//	Stop();
//	
//	vTaskDelay(1000);
//	comClearRxFifo(COM2);
//	Guidance_State = Auto_Wait;
//	vTaskResume(xHandle_AGV_Action);
//	vTaskResume((xHandle_U2_Navigation));
	
	//vTaskResume(xHandle_Right_90Angel_Check);

}
/*******************************************************************************
* Function Name  : void Fork_Straight(void)
* Description    : 分岔路直行
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void Fork_Straight(void)
{
	//int test_buff[300];
	//int buff_count=0;
	//int buff_val =0;
	//Stop();
	vTaskSuspend(xHandle_AGV_Action); 
	Stop();
	if(Direction == Forward )  
	{
		vTaskSuspend((xHandle_U2_Navigation));
	}
	else if(Direction == Back_Up)
	{
		vTaskSuspend((xHandle_U5_Navigation));
	}
	 
	vTaskDelay(1000);
	go_straight();
//	while(1)
//	{
//		/*
//			赋值执行完后再进行buff_count++和buff_val++
//		*/
////		test_buff[buff_count++] = buff_val++;
////		printf("数组为%d",test_buff[buff_count-1]);
////		printf("标号为%d",buff_count);
////		printf("数据为%d",buff_val);
//		
//		//buff_count++;
//		bsp_DelayMS(500);
//		//printf("进入循环");	
//		Read_Speed();
//		Send_Speed();
//	}

}
/*******************************************************************************
* Function Name  : void Fork_Back(void)
* Description    : 分岔路直行
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void Fork_Back(void)
{
//	int test_buff[300];
	//int buff_count=0;
	//int buff_val =0;
	Stop();
	vTaskSuspend(xHandle_AGV_Action); 
	if(Direction == Forward )  
	{
		vTaskSuspend((xHandle_U2_Navigation));
		comClearRxFifo(COM2);
	}
	else if(Direction == Back_Up)
	{
		vTaskSuspend((xHandle_U5_Navigation));
		comClearRxFifo(COM5);
	}
	 
	vTaskDelay(1000);
	go_back();
//	while(1)
//	{
//		/*
//			赋值执行完后再进行buff_count++和buff_val++
//		*/
////		test_buff[buff_count++] = buff_val++;
////		printf("数组为%d",test_buff[buff_count-1]);
////		printf("标号为%d",buff_count);
////		printf("数据为%d",buff_val);
//		
//		//buff_count++;
//		bsp_DelayMS(500);
//		//printf("进入循环");	
//		Read_Speed();
//		Send_Speed();
//	}

}

/*******************************************************************************
* Function Name  : void Read_Speed(void)
* Description    : 读取电机速度
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void Read_Speed(void)
{
	//uint8_t MODH_ReadParam_03H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num)
	
	//读取1号电机的当前速度
	//电机实际转速 = 读取到的速度/10
	if(MODH_ReadParam_03H(1,0x10,1))
	{
//		Read_Speed_1_H = g_tVar.P01;
//		Read_Speed_1_L = g_tVar.P02;
//		Read_Speed_1 = (Read_Speed_1_H<<8+Read_Speed_1_L)/10;
		//printf("左轮速度=%d",g_tVar.P01);
		if(g_tVar.P01>32767)
		{
			g_tVar.P01 = ((65535-g_tVar.P01)+1);
		}	
		//Read_Speed_1 = (g_tVar.P01)/10;
		Read_Speed_1 = (g_tVar.P01);
	}
	else
	{
		//printf("读取左轮速度失败");

	}
	//读取2号电机的当前速度
	//电机实际转速 = 读取到的速度/10
	if(MODH_ReadParam_03H(2,0x10,1))
	{
//		Read_Speed_2_H = g_tVar.P01;;
//		Read_Speed_2_L = g_tVar.P02;
//		Read_Speed_2 = (Read_Speed_2_H<<8+Read_Speed_2_L)/10;
		//printf("右轮速度=%d",g_tVar.P01);
		if(g_tVar.P01>32767)
		{
			g_tVar.P01 = ((65535-g_tVar.P01)+1);
		}	
		//Read_Speed_2 = (g_tVar.P01)/10;
		Read_Speed_2 = (g_tVar.P01);
	}
	else
	{
		//printf("读取右轮速度失败");

	}
	

}
/*******************************************************************************
* Function Name  : void Read_Speed(void)
* Description    : 读取电机速度
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void Send_Speed(void)
{
	//printf("读取结果=%X\r\n",g_tVar.P01);
	//转速显示.左轮转速.val=\"%d\""
	printf("转速显示.左轮转速.val=%d",Read_Speed_1);
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(100);
	
	printf("转速显示.右轮转速.val=%d",Read_Speed_2);
	bsp_DelayMS(100);
	HMI_END();
	
//	printf("转速显示.左轮转速.val=%d\r\n",Read_Speed_1);
//	printf("转速显示.右轮转速.val=%d\r\n",Read_Speed_2);

}



