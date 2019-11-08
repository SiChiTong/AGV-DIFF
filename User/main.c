/*
*********************************************************************************************************
*
����ֱ�������ٶȣ����ý���
����ƫ��1�����㣬ֱ�������ٶ�Ϊ A
��ƫ��2�����㣬ֱ�������ٶ�Ϊ 2A



*
*	
*
*********************************************************************************************************
*/
#include "includes.h"


#include "modbus_host.h"
#include "RFID.h"

#define  Turn_Time     60// ��λs


#define  Auto_Wait     0
#define  Auto_Stop     1
#define  Auto_Straight 2
#define  Auto_Left	   3
#define  Auto_Right    4
#define  Auto_Crossing 5


/*
�ֲ�··��ѡ���־λ
*/
#define Straight_Flag 1
#define Right_Flag    2
#define Left_Flag     3
#define Back_Flag     4

//#define  Safe_Distance 0x64 //��λ������  100cm
#define  Safe_Distance 0x1E //��λ������  30cm

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
											AGV�豸��ʶ
**********************************************************************************************************
*/
#define  ZZBM	0xA3 			//���첿�� ����3
#define  Plant  0x2B 			//����     2�Ż��ӳ���
#define  Num    0x01 			//����     1��AGV
#define  Network_Head_LEN   2	//��ͷ����
#define  Network_End_LEN    1	//��β����

#define  Network_State_LEN   1	//װ���
#define  Network_Place_LEN   1	//�ص㳤��
#define  Network_Reserved_LEN 8
#define  Network_AGV_BUF_LEN  17
 
 
#define Manual_Mode   		0xF1    //�ֶ�ģʽ
#define Auto_Mode	  		0xF2	//�Զ�ģʽ
 
 
 
 
#define Standby   		0xC1    //����
#define Parking	  		0xC2	//վ��ͣ��
#define Moving    		0xC3	//��ʻ��
#define Derail_ERR 		0xC4	//�ѹ챨��
#define Barrier_ERR  	0xC5	//�ϰ��ﱨ��
#define Manual_Stop  	0xC6	//�ϰ��ﱨ��


#define Forward     1 //ǰ������
#define Back_Up     2 //���˷���

#define Straight_Len     10 //ֱ�ж����鳤��
#define Left_Len         10 //��ת�����鳤��
#define Right_Len        10 //��ת�����鳤��
#define Back_Len		 10 //���˶����鳤��


uint8_t Work_Mode ;


uint8_t Network_Head_BUF[Network_Head_LEN] = {0x17,0x18};


uint8_t Network_AGVID_BUF[3]={0,0,0};


uint8_t Network_End_BUF[Network_End_LEN] = {0xFF};

uint8_t Network_Reserved_BUF[Network_Reserved_LEN] = {0x00};



uint8_t A[Network_State_LEN] ={0x33};
uint8_t B[Network_Place_LEN] ={0x44};

uint8_t Place;
uint8_t State  = Standby ;//Ĭ�ϴ��ڴ���״̬

uint8_t Network_AGV_BUF[Network_AGV_BUF_LEN]= {0x00};


uint8_t State_01 =0; //01��̽ͷ״̬����־λ
uint8_t State_16 =0; //16��̽ͷ״̬����־λ

uint8_t Straight_Sets[Straight_Len]   = {0xC1,0xC2,0xC3,0xC4,0xC5};
uint8_t Left_Turn_Sets[Left_Len]      = {0xB1,0xB2,0xB3,0xB4,0xB5};
uint8_t Right_Turn_Sets[Right_Len]    = {0xD1,0xD2,0xD3,0xD4,0xD5};
uint8_t Back_Turn_Sets[Back_Len]      = {0xA1,0xA2,0xA3,0xA4,0xA5};




/*
**********************************************************************************************************
											�¼���־λ�궨��
**********************************************************************************************************
*/
#define BIT_0	(1 << 0) 	//ͣ��
#define BIT_1	(1 << 1)	//��ת
#define BIT_2	(1 << 2)	//��ת
#define BIT_3	(1 << 3)	//ֱ��
#define BIT_4	(1 << 4)	//ֱ��,����⵽�Ź����̽ͷ������7




/*
**********************************************************************************************************
											����������
**********************************************************************************************************
*/

static void AppTaskCreate (void); //���ڴ�������

static void vTask_U2_Navigation(void *pvParameters); //����2 �ŵ�������
static void vTask_U3_RFID(void *pvParameters);		 //����3 RFID����������
static void vTask_U5_Navigation(void *pvParameters); //����5 �ŵ�������
static void vTask_U6_HMI(void *pvParameters);        //����6 HMI�˻���������
//static void vTask_Working(void *pvParameters);     //��������
static void vTaskCheck(void *pvParameters);        	 //�����������


static void vTask_Manual(void *pvParameters);        	//�ֶ���ť
static void vTask_Forward_Barrier(void *pvParameters);        	//�ϰ����ж����� ǰ��

static void vTask_BackUp_Barrier(void *pvParameters);        	//�ϰ����ж�����  ��


///AGV��������vTask_AGV_Action
static void vTask_AGV_Action(void *pvParameters);       

static void vTask_AGV_To_Network(void *pvParameters);       //����AGV״̬����


static void vTask_Left_90Angel_Check(void *pvParameters); //AGV��ת90�ȼ������
static void vTask_Right_90Angel_Check(void *pvParameters); //AGV��ת90�ȼ������







/*
**********************************************************************************************************
											����������
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

	
////��ֵ�ź������
//SemaphoreHandle_t xSemaphore;	//��ֵ�ź������
//QueueHandle_t USART1_Queue;   		
//QueueHandle_t Message_Queue;	

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
**********************************************************************************************************
											������������
**********************************************************************************************************
*/

uint8_t Verify_RFID(uint8_t *data,uint8_t* RFID); //IC����⺯��
void RFID_Check(void); 		//U3  RFID��⺯��
void U2_Navigation_Check(void);//U2  �ŵ�����⺯��
void U5_Navigation_Check(void);//U5  ��������⺯��

void Auto_Work(void);		//��������
void Go_Straight(void);     //ֱ�к���
void Stop(void);			//ͣ������
void Alarm(void);			//��������
void Turn_Left(uint8_t *data);		//��ת����
void Turn_Right(uint8_t *data);		//��ת����
void U2_State(void);		//U2  �ŵ������ݸ�ʽ���״̬������
void U2_State_Check(void);	//U2  �ŵ������ݸ�ʽ���״̬������  ר�����ڳ�ʼ��
void U3_State(void);		//U3  RFID���ݸ�ʽ���״̬������
void U5_State(void);		//U5  ��̬���������ݸ�ʽ��Ⲩ״̬������
void U6_State(void);		//U6  HMI���ݸ�ʽ���״̬������
void U2_Deal(uint8_t *data);//U2  �ŵ������ݴ�����
void U3_Deal(uint8_t *data);//U3  RFID�������ݴ�����
void U5_Deal(uint8_t *data);//U5  �����������ݴ�����
void U5_State_Check(void);	//U2  �ŵ������ݸ�ʽ���״̬������  ר�����ڳ�ʼ��
void U6_Deal(uint8_t *data);//U6  HMI�������ݴ�����
void HMI_END(void);			//HMI������ָ��
void Discern_IC_Card(void);	//IC��ʶ����
void Servo_Init(void);		//�ŷ������ʼ������
void RFID_TO_HMI(uint8_t place); //RFIDʶ��Ŀ�ĵغ�����HMI ����
void Network_Init(void);       	//�����ʼ������

void AGV_To_Network(uint8_t *_Work_Mode,uint8_t *_Reserved);   //ͨ�����緢��AGV״̬����



void Network_Head_Send(void);    //��������ͨѶ��ͷ
void Network_End_Send(void);     //��������ͨѶ��β
void Network_AGVID_Send(void);   //����AGV ID


void Network_AGVState_Send(uint8_t *_State1,uint8_t *_Place1); //����AGV״̬

void Network_Reserved_Send(uint8_t *Reserved); //����AGV��Ϣ��Ԥ����
void Network_AGVWork_Send(uint8_t *_Work_Mode);//����AGV����ģʽ

void AGV_Have_Barrier(void); //��⵽�ϰ���
void AGV_None_Barrier(void); //�ϰ����Ƴ�

void AGV_StopBManual(void);  //�ֶ�ֹͣ����
void AGV_RunBManual(void);	 //�ֶ��ָ�����


void Task_Resume(void); //�ָ�����

void Time_Out(void);//���ת���Ƿ�ʱ

int Path_Choice(uint8_t Order_Place);//�ֲ�·ѡ��
void Fork_Left(void); //�ֲ�·��ת
void Fork_Right(void);//�ֲ�·��ת
void Fork_Straight(void);//�ֲ�·ֱ��
void Fork_Back(void);//�ֲ�·����

void Read_Speed(void);//��ȡ����ٶ�
void Send_Speed(void);//��������ٶȸ�������
void go_straight(void);//ֱ��
void go_back(void);//����
/*
**********************************************************************************************************
											ת�亯������
**********************************************************************************************************
*/
void Left_90Angel(void); //��ת90��
void Right_90Angel(void);//��ת90��

void U2_Left_90Angel_Check(void);  //ͨ��U2���ڵĴŵ�������Ƿ������ת90��
void U2_Right_90Angel_Check(void); //ͨ��U2���ڵĴŵ�������Ƿ������ת90��

void U5_Left_90Angel_Check(void);  //ͨ��U5���ڵĴŵ�������Ƿ������ת90��
void U5_Right_90Angel_Check(void); //ͨ��U5���ڵĴŵ�������Ƿ������ת90��

/*
**********************************************************************************************************
											ָʾ�ƺ�������
**********************************************************************************************************
*/

void Running_Green_Light(void);  //��������ʱ ��ɫ����
void Waiting_Yellow_Light(void); //վ��ͣ���ȴ���������ת��90��ʱ����ɫ����
void Alarm_Red_Light(void);      //���ϱ���ʱ�򣬺�ɫ����

/*
**********************************************************************************************************
											ȫ�ֱ�������
**********************************************************************************************************
*/
uint8_t Guidance_State = 0; //�ŵ���״̬��ͣ������ת����ת��ֱ�У�
uint8_t PLACE_Match    = 0; //�ŵ���״̬��ͣ������ת����ת��ֱ�У�Ŀ�ĵط��ϱ�־λ

////����2	�ŵ���
uint8_t U2_read;
uint8_t U2_ucStatus = 0;  /* ״̬����־ */
uint8_t U2_ucCount=0;
uint8_t U2_buf[20];
///////////////

////����3	RFID������
uint8_t U3_read;
uint8_t U3_ucStatus = 0;  /* ״̬����־ */
uint8_t U3_ucCount=0;
uint8_t U3_buf[20];
///////////////


////����5	�ŵ���
uint8_t U5_read;
uint8_t U5_ucStatus = 0;  /* ״̬����־ */
uint8_t U5_ucCount=0;
uint8_t U5_buf[20];
///////////////

////����6	HMI
uint8_t U6_read;
uint8_t U6_ucStatus = 0;  /* ״̬����־ */
uint8_t U6_ucCount=0;
uint8_t U6_buf[30];
///////////////

uint8_t* RFID_IN;


uint8_t	Manual_Place = 0; //��λ���ֶ�ָ���ص�ģʽ
uint8_t	Auto_Place   = 0; //��λ���Զ�ָ���ص�ģʽ

uint8_t Place_Stop = 0;//Ŀ�ĵر�־λ
uint8_t	Send_First = 0;//HMI��һ�����ݱ�־λ
uint8_t Stop_Flag  = 0; //�Ѿ�ͣ����־λ ��ֹ�ظ����ٶ�ָ��
uint8_t Derail = 0; //�����־λ
uint8_t PLACE_Gone = 0;//��¼�Ѿ��ﵽĿ�ĵ�����

uint8_t	Left  = 0;//�Ѿ���ת��־λ ��ֹ�ظ����ٶ�ָ��
uint8_t	Right = 0;//�Ѿ���ת��־λ ��ֹ�ظ����ٶ�ָ��
uint8_t	Stop_change = 0;

uint8_t UART2_Done=0; 	//����2���һ�����ݴ���
uint8_t UART3_Done =0;	//����3���һ�����ݴ���
uint8_t UART5_Done =0;	//����5���һ�����ݴ���
uint8_t UART6_Done =0;	//����6���һ�����ݴ���


uint8_t U2_ucStatus_Check = 0; 
uint8_t U5_ucStatus_Check = 0; 
uint8_t U2_Navigation_Checked =0;
uint8_t U5_Navigation_Checked =0;
uint8_t U2_Navigation_Correct = 0;//����2 �ŵ���������    ���ɹ���־λ
uint8_t U5_Navigation_Correct = 0;//����5 �ŵ���������    ���ɹ���־λ

uint8_t Attitude_Correct = 0; //����5 ��̬������  ���ɹ���־λ

uint8_t HMI_Command[30];//����HMIָ�������

uint8_t Barrier = 66; //�ϰ����־λ

uint8_t new_data1;
uint8_t new_data2;

/*





�ٶȶ��壬���٣�����ٶ�

�ٶȲ� = �������ת�� - ��������ת��





*/
uint16_t Speed_Straight = 320; //ֱ��ʱ�������ת��  ��������ٶ���400

uint16_t Speed_Difference  = 12; //�ٶȲ� ���������ٶ���400-96=304  16�Ƚ��ȶ�

uint16_t Speed_90Angle = 90;//����תת90�ȵ��ٶ�   ��������ٶ���90

uint16_t Speed_90Angle_Difference = 30;//����תת90�ȵ��ٶ�    ���������ٶ���90-30=60

uint16_t Speed_Straight_GO = 90;//����ֱ��ʱ����ٶ�

uint16_t HMI_Straight_Speed   = 0;  //ֱ���ٶ�
uint16_t HMI_Correction_Speed = 0;	//ֱ�������ٶ�
uint16_t HMI_Turn_90_Speed    = 0;  //����ת90���ٶ�



uint16_t Motor1_Speed = 0;  //���� 1�ŵ���ٶ�
uint16_t Motor2_Speed = 0;	//���� 2�ŵ���ٶ�

uint16_t Direction = Forward; //Ĭ���Ƿ�����ǰ�����򣬻�Ӱ��������ת

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int main(void)
{
	/* 
	  ����������ǰ��Ϊ�˷�ֹ��ʼ��STM32����ʱ���жϷ������ִ�У������ֹȫ���ж�(����NMI��HardFault)��
	  �������ĺô��ǣ�
	  1. ��ִֹ�е��жϷ����������FreeRTOS��API������
	  2. ��֤ϵͳ�������������ܱ���ж�Ӱ�졣
	  3. �����Ƿ�ر�ȫ���жϣ���Ҹ����Լ���ʵ��������ü��ɡ�
	  ����ֲ�ļ�port.c�еĺ���prvStartFirstTask�л����¿���ȫ���жϡ�ͨ��ָ��cpsie i������__set_PRIMASK(1)
	  ��cpsie i�ǵ�Ч�ġ�
     */
	__set_PRIMASK(1);  
	
	/* Ӳ����ʼ�� */
	bsp_Init(); 
	
	PE1_IN2_NVIC(OPEN);
	PE2_IN3_NVIC(OPEN);
	
	/* �������� */
	AppTaskCreate();
	
	
    /* �������ȣ���ʼִ������ */
    vTaskStartScheduler();

	/* 
	  ���ϵͳ���������ǲ������е�����ģ����е����Ｋ�п��������ڶ�ʱ��������߿��������
	  heap�ռ䲻����ɴ���ʧ�ܣ���Ҫ�Ӵ�FreeRTOSConfig.h�ļ��ж����heap��С��
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 30 * 1024 ) )
	*/
	while(1);
}


/*

*********************************************************************************************************
*	�� �� ��: vTask_AGV_To_Network
*	����˵��: 	
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2  

*********************************************************************************************************

*/

static void vTask_AGV_To_Network(void *pvParameters) //�ŵ���
{
	TickType_t xLastWakeTime;const TickType_t xFrequency = 5000; //5s
	xLastWakeTime = xTaskGetTickCount();
	
	while (1)
	{
		//AGV_To_Network(&Work_Mode,&Network_Reserved_BUF[0]);    //ͨ�����緢��AGV״̬����
		AGV_To_Network(&Work_Mode,&Network_Reserved_BUF[0]);    //ͨ�����緢��AGV״̬����
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}





/*

*********************************************************************************************************
*	�� �� ��: vTask_Right_90Angel_Check
*	����˵��: 	
*	��    ��: ����Ƿ������ת90������
*	�� �� ֵ: ��
*   �� �� ��: 2  

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
*	�� �� ��: vTask_Left_90Angel_Check
*	����˵��: 	
*	��    ��: ����Ƿ������ת90��
*	�� �� ֵ: ��
*   �� �� ��: 2  

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
*	�� �� ��: vTask_Forward_Barrier
*	����˵��: �������ϰ�������   ǰ��	
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2  

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
									&ulValue, /* ���� ulNotifiedValue ������ ulValue �� */
									portMAX_DELAY); /* ��������ӳ�ʱ�� */
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
*	�� �� ��: vTask_BackUp_Barrier
*	����˵��: �������ϰ�������  ��
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2  

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
									&ulValue, /* ���� ulNotifiedValue ������ ulValue �� */
									portMAX_DELAY); /* ��������ӳ�ʱ�� */
		
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
*	�� �� ��: vTask_Manual
*	����˵��: �ֶ�ֹͣ��ť
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2  

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
									&ulValue, /* ���� ulNotifiedValue ������ ulValue �� */
									portMAX_DELAY); /* ��������ӳ�ʱ�� */
		if( xResult == pdPASS )
		{
			/* ���յ���Ϣ������Ǹ�λ������ */
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
*	�� �� ��: vTask_U2_Navigation
*	����˵��: ����2 �ŵ������� ��������	
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2  

			�ŵ��� ���ݸ�ʽ��һ֡����5������				
			��1��	��2��	��3��	��4��	��5��
			0x55	0x54	0x09	DATA	~DATA
			HEAD1	HEAD2	NUM	��������ֵ	ȡ��

����2�ŵ���
*********************************************************************************************************

*/
static void vTask_U2_Navigation(void *pvParameters) //�ŵ���
{

	while (1)
	{
		UART2_Done = 0;
		U2_State( ); //״̬��

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
*	�� �� ��: vTask_U3_RFID
*	����˵��: ����3 RFID����������	
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 3  
*********************************************************************************************************
*/
static void vTask_U3_RFID(void *pvParameters)//������
{
	
    while(1)
    {
		
		UART3_Done = 0;
		U3_State( ); //״̬��
		vTaskDelay(5);
    }
}


/*
*********************************************************************************************************
*	�� �� ��: vTask_U5_Navigation
*	����˵��: ����5 �ŵ���������	
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 3  
*********************************************************************************************************
*/

static void vTask_U5_Navigation(void *pvParameters)
{
  	while (1)
	{

		UART5_Done = 0;
		U5_State( ); //״̬��
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
*	�� �� ��: vTask_U6_HMI
*	����˵��: ����6 HMI�˻���������	
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 3  
*********************************************************************************************************
*/
static void vTask_U6_HMI(void *pvParameters)
{
    while(1)
    {
		U6_State( ); //״̬��
		vTaskDelay(20);
    }
}







/*
*********************************************************************************************************
*	�� �� ��: vTaskCheck
*	����˵��: ��������Ƿ�����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 8 
*   �����ʼ��
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
		AGV������ʼ��
		*/

		AGV_To_Network(&Work_Mode,&Network_Reserved_BUF[0]);    //ͨ�����緢��AGV״̬����
		
		bsp_DelayMS(1000);
		/*
		RFID���������
		*/
		
		//RFID_Check();
		
		
		/*
		�ŵ�����������⣬�����������ϴ�����ģʽ
		���ɹ���رմ���2�����жϣ�
		��ֹFIFO���
		��HMI�˻������´���ȷָ���������������ж�
		*/
		
		
		U2_Navigation_Check();
		
		/*
		�ŵ�����������⣬�����������ϴ�����ģʽ
		���ɹ���رմ���5�����жϣ�
		��ֹFIFO���
		��HMI�˻������´���ȷָ���������������ж�
		*/
		RFID_Check();


	    //U5_Navigation_Check();
		//USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); 
		
		
		/*
		��������������⣬�����������ϴ�����ģʽ
		���ɹ���رմ���5�����жϣ�
		��ֹFIFO���
		��HMI�˻������´���ȷָ���������������ж�
		*/
		//Attitude_Check();
		//USART_ITConfig(UART5,USART_IT_RXNE,DISABLE);

		/*
		�ŷ������⣬485MODBUSЭ��
		*/
		Servo_Init();
	
		printf("���Դ���.AGV����.val=1");
		bsp_DelayMS(100);
		HMI_END();
		bsp_DelayMS(1000);
		
		///�������ִ����Ϻ�ɾ��������񣬲���ִ�д�����
		////�رմ��ڽ����жϣ��������FIFO�����ݣ��Ա���������
		USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); //�رմ���2�Ĵŵ�������

		
		//Ĭ�Ϲ��𴮿�5
		vTaskSuspend(xHandle_U5_Navigation);  //���𴮿�5 �ŵ���
		
		USART_ITConfig(UART5,USART_IT_RXNE,DISABLE); //�رմ���5�Ĵŵ�������
		

		////�رմ��ڽ����жϣ��������FIFO�����ݣ��Ա���������
		comClearRxFifo(COM2);
		comClearRxFifo(COM3);
		comClearRxFifo(COM5);
			
	
		///�������ֱ��HMI�յ�ָ��Żָ�����	
		vTaskSuspend(xHandle_AGV_Action); 
		vTaskSuspend(xHandle_Left_90Angel_Check);
		vTaskSuspend(xHandle_Right_90Angel_Check);
		//vTaskSuspend(xHandle_BackUp_Barrier);
		vTaskSuspend(xHandlevTaskCheck);
	}
	
}





/*

*********************************************************************************************************
*	�� �� ��: vTask_AGV_Action
*	����˵��: AGV��������,һֱ��������״̬�ȴ��Ŵ������źţ��������źŶ���	
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��:   

*********************************************************************************************************

*/

static void vTask_AGV_Action(void *pvParameters)
{
	

	BaseType_t Navigation;
	uint32_t ulValue;
	
	while(1)
	{		
		//����portMAX_DELAY,������һֱ��������״̬��һ���ӵ����ŵ����������źţ������Ϲ�����ʵ��ʵʱ��
		
		
		//��һ������  �����0xFFFFFFFF ʹ��֪֮ͨǰ������
		//�ڶ�������  �����0xFFFFFFFF �ں���xTaskNotifyWait()�˳�ǰ������
		// 0x00000000
		Navigation = xTaskNotifyWait(0xFFFFFFFF,      
						          0xFFFFFFFF,      
						          &ulValue,        /* ����ulNotifiedValue������ulValue�� */
						          portMAX_DELAY);  /* ��������ӳ�ʱ�� */
		
		if( Navigation == pdPASS ) 
		{
			/*  
				���յ���Ϣ�����AGV��Ҫִ�еĶ��� 
			    ���ó�if else ifΪ��ʵ�ֻ�����ͣ����ִ�С���ת����ת�Ļ���
			    AGV ͣ��
			
				���ڶ��ֽṹ��˵
				�����ֲַ�ڵ�ʱ������if else if �߼���
				С����������ת����ת��������ֱ�У�
				����������ʵ�ּ򵥵�·��ѡ��
				�������Ҫ���ӹ��ܵĻ��������ڷֲ��ǰ������IC����С����ǰ���ٺ�ѡ��·��
			
				����˫�ֲ��ٽṹ��ת����ԭ����ת
				
			
			*/
			/* AGV �ѹ� */
			if((ulValue & BIT_4) != 0 ) 
			{
				Go_Straight();
				/* 
				��������ʱ�̵���
				*/
				Running_Green_Light();			
				if(Derail)
				{
					bsp_DelayMS(100);
					printf("��ʻ����.�ѹ�.val=0"); //���»ص����
					bsp_DelayMS(100);
					HMI_END();
					Derail = 0;
				}	
			}
			else if((ulValue & BIT_0) != 0 ) 
			{
				Stop();
				bsp_DelayMS(100);
				printf("��ʻ����.�ѹ�.val=1"); //�ѹ�
				bsp_DelayMS(100);
				HMI_END();
				Derail = 1;
				/* 
				�ѹ�ʱ���������
				*/
				Alarm_Red_Light();
				
			}		
			/* AGV ��ת  */
			else if((ulValue & BIT_1) != 0)
			{
				Turn_Right(&U2_buf[0]);
				/* 
				��������ʱ�̵���
				*/
				Running_Green_Light();			
				if(Derail)
				{
					bsp_DelayMS(100);
					printf("��ʻ����.�ѹ�.val=0"); //���»ص����
					bsp_DelayMS(100);
					HMI_END();
					Derail = 0;
				}
			}
			/* AGV ��ת  */
			else if((ulValue & BIT_2) != 0) 
			{
				Turn_Left(&U2_buf[0]);
				/* 
				��������ʱ�̵���
				*/				
				Running_Green_Light();
				if(Derail)
				{
					bsp_DelayMS(100);
					printf("��ʻ����.�ѹ�.val=0"); //���»ص����
					bsp_DelayMS(100);
					HMI_END();
					Derail = 0;
				}
			}
			/* AGV ֱ�� */
			else if((ulValue & BIT_3) != 0)
			{

				Go_Straight();
				/* 
				��������ʱ�̵���
				*/
				Running_Green_Light();
				if(Derail)
				{
					bsp_DelayMS(100);
					printf("��ʻ����.�ѹ�.val=0"); //���»ص����
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
*	�� �� ��: AppTaskCreate
*	����˵��: ����Ӧ������
*	��    �Σ���
*	�� �� ֵ: ��


��������
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
	taskENTER_CRITICAL();           //�����ٽ���
	
/////����AGV״̬����
	xTaskCreate( vTask_AGV_To_Network,   	/* ������  */
                 "vTask_AGV_To_Network",     	/* ������    */
                 512,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 1,                 	/* �������ȼ�*/
                 &xHandle_AGV_To_Network  );  /* ������  */
	
/////����2 �ŵ������������ݴ�������	
	xTaskCreate( vTask_U2_Navigation,   	/* ������  */
                 "vTask_U2_Navigation",     	/* ������    */
                 512,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 2,                 	/* �������ȼ�*/
                 &xHandle_U2_Navigation  );  /* ������  */
	
/////����3 RFID���������ݴ�������	
	xTaskCreate( vTask_U3_RFID,    		/* ������  */
                 "vTask_U3_RFID",  		/* ������    */
                 512,         		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,        		/* �������  */
				 7,           		/* �������ȼ�*/
                 &xHandle_U3_RFID  ); /* ������  */
				 
/////����5 �ŵ������������ݴ�������	
	xTaskCreate( vTask_U5_Navigation,     		/* ������  */
                 "vTask_U5_Navigation",   		/* ������    */
                 512,             		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 2,               		/* �������ȼ�*/
                 &xHandle_U5_Navigation);  /* ������  */
				 
/////����6 ���������ݴ�������				 
	xTaskCreate( vTask_U6_HMI,     		/* ������  */
                 "vTask_U6_HMI",   		/* ������    */
                 512,             		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		/* �������  */
                 5,               		/* �������ȼ�*/
                 &xHandle_U6_HMI   );  /* ������  */				 
				 
/////����������	
	xTaskCreate( vTaskCheck,     		/* ������  */
				 "vTaskCheck",   		/* ������    */
				 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
				 NULL,           		/* �������  */
				 8,              		/* �������ȼ�*/
				 &xHandlevTaskCheck );   /* ������  */		 
				 
			 
				 
				 
/////�ϰ�������	 ǰ��		
	xTaskCreate( vTask_Forward_Barrier,     		/* ������  */
				 "vTask_Forward_Barrier",		/* ������    */
				 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
				 NULL,           		/* �������  */
				 6,              		/* �������ȼ�*/
				 &xHandle_Forward_Barrier   );   /* ������  */		
				 
				 
/////�ϰ�������	 ��		
	xTaskCreate( vTask_BackUp_Barrier,     		/* ������  */
				 "vTask_BackUp_Barrier",		/* ������    */
				 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
				 NULL,           		/* �������  */
				 6,              		/* �������ȼ�*/
				 &xHandle_BackUp_Barrier     );   /* ������  */					 
				 
				 
				 
/////�ֶ�ͣ������������					 
	xTaskCreate( vTask_Manual,     		/* ������  */
				 "vTask_Manual",		/* ������    */
				 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
				 NULL,           		/* �������  */
				 5,              		/* �������ȼ�*/
				 &xHandle_Manual );   /* ������  */				 
				 
				 

////////////AGV��������  
//	xTaskCreate( vTask_AGV_Action,     		/* ������  */
//				 "vTask_AGV_Stop",   		/* ������    */
//				 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
//				 NULL,           		/* �������  */
//				 7,              		/* �������ȼ�*/
//				 &xHandle_AGV_Action);   /* ������  */
				 
	xTaskCreate( vTask_AGV_Action,     		/* ������  */
				 "vTask_AGV_Stop",   		/* ������    */
				 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
				 NULL,           		/* �������  */
				 4,              		/* �������ȼ�*/
				 &xHandle_AGV_Action);   /* ������  */		
		 
				 
////////////AGV��ת90�ȼ������ 			 
	xTaskCreate( vTask_Left_90Angel_Check,     		/* ������  */
				 "vTask_Left_90Angel_Check",   		/* ������    */
				 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
				 NULL,           		/* �������  */
				 3,              		/* �������ȼ�*/
				 &xHandle_Left_90Angel_Check );   /* ������  */

////////////AGV��ת90�ȼ������
	xTaskCreate( vTask_Right_90Angel_Check,     		/* ������  */
				 "vTask_Right_90Angel_Check",   		/* ������    */
				 512,            		/* ����ջ��С����λword��Ҳ����4�ֽ� */
				 NULL,           		/* �������  */
				 3,              		/* �������ȼ�*/
				 &xHandle_Right_90Angel_Check );   /* ������  */			 		 
				 
											 
	taskEXIT_CRITICAL();            //�˳��ٽ���							 
								 
								 
}


/*********************************************************************************************************
*	�� �� ��: Servo_Init
*	����˵��: ��ʼ��2���ŷ������1.Modbus ʹ�� 2.���������ʹ�� 3.��ȡ��������
*	��    ��: Pulse
*	�� �� ֵ: ��
*   ��ַ �������� 		ֻ��/��д 	     ������Χ 			����˵��
     0   Modbus ʹ��       ��д 			    0~1             0��modbus ��ֹ
															1��modbus ʹ��
															
	 1	 ���������ʹ��     ��д			    0~1	            0�������������ֹ
															1�����������ʹ��
	
	 2   ���Ŀ���ٶ�       ��д           0~3000 r/min      �ٶ�ģʽʱ��Ŀ���ٶ�
															λ��ģʽʱ������ٶ�
															
	 3   ������ٶ�         ��д           0~30000(r/min)/s   ������ٶ�																


*   
*********************************************************************************************************/
void Servo_Init(void)
{
	//1.д �����ŷ���  �ӻ���ַ1�������� 06 �Ĵ�����ַ 0x00  Modbus ʹ�ܣ� 1
	//2.д ת���ŷ���  �ӻ���ַ2�������� 06 �Ĵ�����ַ 0x00  Modbus ʹ�ܣ� 1
	//MODH_WriteParam_06H(1,0,1);   

	printf("���Դ���.��ʾ��.txt=\"�ŷ������������ʼ����\"");
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(500);

	while(MODH_WriteParam_06H(1,0,1) != 1)
	{
		printf("���Դ���.��ʾ��.txt=\"��1���ŷ����Modbusʹ����\"");
		bsp_DelayMS(100);
		HMI_END();
		bsp_DelayMS(100);

	}
	bsp_DelayMS(500);
	printf("���Դ���.��ʾ��.txt=\"��1���ŷ����Modbusʹ�ܳɹ�\"");
	bsp_DelayMS(100);
	HMI_END();

	
	
	
	while(MODH_WriteParam_06H(2,0,1)!=1)
	{
		printf("���Դ���.��ʾ��.txt=\"��2���ŷ����Modbusʹ����\"");
		bsp_DelayMS(100);
		HMI_END();
		bsp_DelayMS(100);

	}
	printf("���Դ���.��ʾ��.txt=\"��2���ŷ����Modbusʹ�ܳɹ�\"");
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(1000);
	
}



/*
�����õĺ���
*/
/*********************************************************************************************************
*	�� �� ��: U2_State
*	����˵��: �Ŵ������ϴ������ݰ�ͷ��֤״̬��
*	��    ��: Pulse
*	�� �� ֵ: ��   



ʮ��·�ŵ������ݣ�55 54 data1 data2 CRC
data1 01~08
data1 09~16
*********************************************************************************************************/
void U2_State(void)
{
	//const uint8_t CDH_HEAD[3]= {0x55,0x54,0x09}; //�������ݵİ�ͷ 5���ֽ� ��·�ŵ���
	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //�������ݵİ�ͷ 5���ֽ�    ʮ��·�ŵ���
	
	if (comGetChar(COM2, &U2_read))
	{
		switch (U2_ucStatus)
		{
			/* ״̬0��֤���յ�0x01 */
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
*	�� �� ��: U2_State_Check
*	����˵��: �Ŵ������ϴ������ݰ�ͷ��֤״̬�������ڿ����Ŵ��������
*	��    ��: Pulse
*	�� �� ֵ: ��   
*********************************************************************************************************/
void U2_State_Check(void)
{
	//const uint8_t CDH_HEAD[3]= {0x55,0x54,0x09}; //�������ݵİ�ͷ 5���ֽ�  ��·�ŵ���
	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //�������ݵİ�ͷ 5���ֽ� ʮ��·�ŵ���
	if (comGetChar(COM2, &U2_read))
	{
		switch (U2_ucStatus_Check)
		{
			/* ״̬0��֤���յ�0x01 */
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
*	�� �� ��: U2_Deal
*	����˵��: ���ݴŴ�����̽ͷ������ݷ��Ͷ�Ӧ���ź����� vTask_AGV_Action
*	��    ��: uint8_t* data   
*	�� �� ֵ: ��   
	AGV��������
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
	
		��01~05̽ͷ����������ת
		
		��12~16̽ͷ����������ת
	*/
	/*
		ͳ�ƴŴ�������⵽������̽ͷ��
	*/
	//��������к�1�������㷨
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
		Ϊ��ʮ��·�ڴ�ת�� 
		�ŵ�����Ӧ����̽ͷ������7����ֱ��
	*/
	
	if((data1_num + data2_num) >=7)
	{
		if(Guidance_State != Auto_Crossing)
		{	
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_4,             /* ����Ŀ�������¼���־λbit0  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_0���л������ 
														  �������ֵ���¼���־λ��*/
			Guidance_State = Auto_Crossing;	
			State = Moving; //��ʻ״̬
		}	
	}

	/* �ѹ� */
	else if(*data == 0x00 & *(data+1) ==0x00)//�ж�������������ͣ�� 0000 0000 
	{
		if(Guidance_State != Auto_Stop)
		{	
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_0,             /* ����Ŀ�������¼���־λbit0  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_0���л������ 
														  �������ֵ���¼���־λ��*/
			Guidance_State = Auto_Stop;	
			State = Derail_ERR; //�ѹ�״̬
		}	
	}
	
	
	/* ��ת */
	else if((*data & 0xFC))  //�Ŵ��� 0xF0=1111 0000  �м���0xF8 =1111 1000  ����С���� 0xFC = 1111 1100
	{
		//if(Guidance_State != Auto_Right)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_1,             /* ����Ŀ�������¼���־λbit1  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_1���л������ 
														  �������ֵ���¼���־λ��*/
			Guidance_State = Auto_Right;
			State = Moving; //��ʻ״̬
		}
	}
	
	/* ��ת */
	else if(*(data+1) & 0x3F) //�Ŵ��� 0x0F=0000 1111 �м���0x1F =0001 1111 ����С���� 0x3F = 0011 1111
	{
		//if(Guidance_State != Auto_Left)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_2,             /* ����Ŀ�������¼���־λbit2  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_2���л������ 
														  �������ֵ���¼���־λ��*/
			Guidance_State = Auto_Left;
			State = Moving; //��ʻ״̬
		}		
	}
	
	/* ֱ�� */
	else   
	{
		if(Guidance_State != Auto_Straight)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_3,             /* ����Ŀ�������¼���־λbit3  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_3���л������ 
														  �������ֵ���¼���־λ��*/
				
			Guidance_State = Auto_Straight;
			State = Moving; //��ʻ״̬
		}

	}	
	
	
}


/*********************************************************************************************************
*	�� �� ��: U3_State
*	����˵��: RFID�ϴ������ݰ�ͷ��֤״̬��
*	��    ��:   
*	�� �� ֵ: ��   
*********************************************************************************************************/
void U3_State(void) //RFID
{

	const uint8_t RFID_HEAD[5]= {0x04,0x16,0x03,0x20,0x00}; //�������ݵİ�ͷ 5���ֽ�
	if (comGetChar(COM3, &U3_read))
	{
		switch (U3_ucStatus)
		{
			/* ״̬0��֤���յ�0x04 */
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
			/* ״̬1��֤���յ�0x16 */
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
			/* ״̬1��֤���յ�0x03 */
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
			/* ״̬1��֤���յ�0x20 */	
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
			/* ״̬1��֤���յ�0x00 */	
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
				 /* ���չ�3������ */
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
*	�� �� ��: U3_Deal
*	����˵��: �ѵ�ַ��RFID_IN
*	��    ��: uint8_t* data   
*	�� �� ֵ: ��   
*********************************************************************************************************/
void U3_Deal(uint8_t* data)
{
	//uint8_t *RFID_IN[20];
	RFID_IN = data; //�ѵ�ַ��RFID_IN
	
}


/*********************************************************************************************************
*	�� �� ��: Verify_RFID
*	����˵��: ����RFID�����������Ŀ�Ƭ������HMI��Ŀ�ĵ�ָ������жϣ��жϵ�ǰ�ص��Ƿ�ΪĿ�ĵ�
*	��    ��: uint8_t* data ��uint8_t* RFID  
*	�� �� ֵ: ��   
*********************************************************************************************************/
uint8_t Verify_RFID(uint8_t* data,uint8_t* RFID)
{
	int i;
	for(i = 0;i <3;i++)
	{
		if(*data != *RFID) //��֤�ص�
		{
			PLACE_Match=0; //����ָ��Ŀ�ĵ�	
			break;
		}
		else
		{
			data++;
			RFID++;
			PLACE_Match=1;//��ָ��Ŀ�ĵ�	
		}
	}	
	if(PLACE_Match)
	{

		/*
			ͣ���ȴ�ʱ���ɫ����
		*/
		Waiting_Yellow_Light();
	}
	return PLACE_Match;

}
/*********************************************************************************************************
*	�� �� ��: U5_State
*	����˵��: ����5 �ŵ��������������ݰ�ͷ��֤״̬��
*	��    ��:   
*	�� �� ֵ: ��   
*********************************************************************************************************/
void U5_State(void)
{
	//const uint8_t CDH_HEAD[3]= {0x55,0x54,0x09}; //�������ݵİ�ͷ 5���ֽ� ��·�ŵ���
	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //�������ݵİ�ͷ 5���ֽ�    ʮ��·�ŵ���
	
	if (comGetChar(COM5, &U5_read))
	{
		switch (U5_ucStatus)
		{
			/* ״̬0��֤���յ�0x01 */
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
				if(U5_read!=0x55) ///�����и�BUG,��һ������Ϊ0X55
				{
					U5_buf[U5_ucCount] = U5_read;              
					 /* ���չ�2������ */
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
*	�� �� ��: U5_Deal
*	����˵��: ������̬��������⵽�����ݽ����жϣ���Z��ǶȵĻ�ȡ���ж�
*	�� �� ֵ: ��   
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
	
		��01~05̽ͷ����������ת
		
		��12~16̽ͷ����������ת
	*/
	/*
		ͳ�ƴŴ�������⵽������̽ͷ��
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
		Ϊ��ʮ��·�ڴ�ת�� 
		�ŵ�����Ӧ����̽ͷ������7����ֱ��
	*/
	
	if((data1_num + data2_num) >=7)
	{
		if(Guidance_State != Auto_Crossing)
		{	
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_4,             /* ����Ŀ�������¼���־λbit0  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_0���л������ 
														  �������ֵ���¼���־λ��*/
			Guidance_State = Auto_Crossing;	
			State = Moving; //��ʻ״̬
		}	
	}

	/* �ѹ� */
	else if(*data == 0x00 & *(data+1) ==0x00)//�ж�������������ͣ�� 0000 0000 ���� ��վ��ͣ�� 1111 1111
	{
		if(Guidance_State != Auto_Stop)
		{	
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_0,             /* ����Ŀ�������¼���־λbit0  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_0���л������ 
														  �������ֵ���¼���־λ��*/
			Guidance_State = Auto_Stop;	
			State = Derail_ERR; //�ѹ�״̬
		}	
	}
	
	
	/* ��ת */
	else if((*data & 0xF8))    
	{
		if(Guidance_State != Auto_Right)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_1,             /* ����Ŀ�������¼���־λbit1  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_1���л������ 
														  �������ֵ���¼���־λ��*/
			Guidance_State = Auto_Right;
			State = Moving; //��ʻ״̬
		}
	}
	
	/* ��ת */
	else if(*(data+1) & 0x1F)    	
	{
		if(Guidance_State != Auto_Left)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_2,             /* ����Ŀ�������¼���־λbit2  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_2���л������ 
														  �������ֵ���¼���־λ��*/
			Guidance_State = Auto_Left;
			State = Moving; //��ʻ״̬
		}		
	}
	
	/* ֱ�� */
	else   
	{
		if(Guidance_State != Auto_Straight)
		{
			xTaskNotify(xHandle_AGV_Action, 				/* Ŀ������ */
									BIT_3,             /* ����Ŀ�������¼���־λbit3  */
									eSetBits);         /* ��Ŀ��������¼���־λ��BIT_3���л������ 
														  �������ֵ���¼���־λ��*/
				
			Guidance_State = Auto_Straight;
			State = Moving; //��ʻ״̬
		}
				
		
	}	
	
}

	
	
	
	


/*********************************************************************************************************
*	�� �� ��: U6_State
*	����˵��: HMI�������ϴ������ݰ�ͷ��֤״̬��
*	��    ��:   
*	�� �� ֵ: ��   
*********************************************************************************************************/
void U6_State(void)
{
	const uint8_t HMI_HEAD[2]={0x55,0x54};//HMI���ݰ�ͷ 
	if (comGetChar(COM6, &U6_read))
	{
		switch (U6_ucStatus)
		{
			/* ״̬0��֤���յ�0x04 */
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
			 /* ״̬1��֤���յ�0x16 */
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
				 /* ���չ�24������ 

				*/
				if(U6_ucCount == 24-1)
				{	
					if(U6_read == 0xFF) //��֤��β
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
*	�� �� ��: U6_Deal
*	����˵��: ����HMI�������ϴ������ݽ��д���
*	�� �� ֵ: ��   
*********************************************************************************************************/
void U6_Deal(uint8_t* data)
{
/*
*(data)   ������
	
*(data+1)	A1      *(data+6)	B1		*(data+11)	C1		*(data+16)	D1		*(data+21)	�յ�
*(data+2)	A2      *(data+7)	B2		*(data+12)	C2		*(data+17)	D2		*(data+22)	���
*(data+3)	A3		*(data+8)	B3		*(data+13)	C3		*(data+18)	D3
*(data+4)	A4		*(data+9)	B4		*(data+14)	C4		*(data+19)	D4
*(data+5)	A5		*(data+10)	B5		*(data+15)	C5		*(data+20)	D5
*/
//	USART_ITConfig(UART5,USART_IT_RXNE,DISABLE); //�رճ��������ϴ���
//	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); //�رմŵ�������
	
	uint8_t place_i;
	uint8_t ID_i;
	uint8_t HMI_Straight_Speed_L;
	uint8_t HMI_Straight_Speed_H;
	switch(*data) //������
	{
		case 0xF1: //��λ���ֶ�ģʽ
		{
//			Manual_Place = 1; //��λ���ֶ�ָ���ص�ģʽ
//			Auto_Place = 0;	
			Work_Mode = Manual_Mode;
			printf("�յ��ֶ�ģʽ.val=1");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);
			
			for(place_i=0;place_i<26;place_i++)
			{
				HMI_Command[place_i]=*(data+1);
				//printf("HMI:%X",*(data+1));
				data++;
				//printf("���ݣ�%x\r\n",HMI_Command[place_i]);
			}
			
			PLACE_Gone = 0;
			while(HMI_Command[PLACE_Gone]==0x00)	//�޳���ȥ��Ŀ�ĵ�
			{
				PLACE_Gone++;
			}
			
			/*
				����Ŀ�ĵ�
			*/
			Place = HMI_Command[PLACE_Gone];
			
			/*
				�ָ�����
			*/
			Task_Resume();
			
			break;
		}
		case 0xF2://��λ���Զ�ģʽ
		{
//			Auto_Place =1; //��λ���Զ�ָ���ص�ģʽ
//			Manual_Place = 0;
			Work_Mode = Auto_Mode;
			printf("�յ��Զ�ģʽ.val=1");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);
			for(place_i=0;place_i<26;place_i++)
			{
				HMI_Command[place_i]=*(data+1);
				data++;
			}	
			
			PLACE_Gone = 0;
			while(HMI_Command[PLACE_Gone]==0x00)	//�޳���ȥ��Ŀ�ĵ�
			{
				PLACE_Gone++;
			}
			
			/*
				����Ŀ�ĵ�
			*/
			Place = HMI_Command[PLACE_Gone];
			/*
				�ָ�����
			*/
			Task_Resume();
						
			break;		
		}			
		
		case 0xF3://ͣ��ʱ�䵽����������ʱ�� 
		{
			
			Place_Stop=0; //����������־λ
			bsp_DelayMS(300);
			printf("ͣ��ʱ�����.val=1");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);
			while(HMI_Command[PLACE_Gone]==0x00)	//�޳���ȥ��Ŀ�ĵ�
			{
				PLACE_Gone++;
			}
			
			/*
				����Ŀ�ĵ�
			*/
			Place = HMI_Command[PLACE_Gone];
			
			/*����ͣվ�󣬹��� vTask_U3_RFID
			  ֱ��HMI�´�ͣ��ʱ�����ָ��Żָ�
			*/

//			vTaskResume(xHandle_U2_Navigation);
//			vTaskResume(xHandle_U5_Attitude);
//			vTaskResume(xHandle_U3_RFID);
			/*
				�ָ�����
			*/
			Task_Resume();
				
			break;
		}
		case 0xF4://����Ŀ�ĵص������ͣ������ģʽ 
		{
			
			
			Work_Mode = 0 ;//
			State  = Standby ;//Ĭ�ϴ��ڴ���״̬
			Place = 0;
			bsp_DelayMS(300);
			printf("�������ģʽ.val=1");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);

			USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); //�����������USART_IT_RXNE���ж��ź� �ŵ���
			//USART_ITConfig(USART3,USART_IT_RXNE,ENABLE); //�����������USART_IT_RXNE���ж��ź� RFID
			USART_ITConfig(UART5, USART_IT_RXNE,DISABLE);  //�����������USART_IT_RXNE���ж��ź� ������

	
			break;
		}
		case 0xF6://HMI����AGV ID
		{
			
			
			Work_Mode = 0 ;//
			State  = Standby ;//Ĭ�ϴ��ڴ���״̬
			Place = 0;
//			bsp_DelayMS(300);
//			printf("�������ģʽ.val=1");
//			bsp_DelayMS(300);
//			HMI_END();
//			bsp_DelayMS(100);
		    /*
				��ȡ��� �ٶ�
			*/
			for(ID_i=0;ID_i<3;ID_i++)
			{
				Network_AGVID_BUF[ID_i]=*(data+1+ID_i);
			}
			
			break;
		}
		case 0xF7://HMI���͵���ٶ�
		{
			
			
			Work_Mode = 0 ;//
			State  = Standby ;//Ĭ�ϴ��ڴ���״̬
			Place = 0;
			
			/*
			ֱ���ٶ�      	2�ֽ�
			ֱ�������ٶ�		1�ֽ�
			����ת90���ٶ�	1�ֽ�
			*/
			
			HMI_Straight_Speed_L = *(data+1); //ֱ���ٶȵĵ�λ
			HMI_Straight_Speed_H = *(data+2); //ֱ���ٶȵĸ�λ
			HMI_Straight_Speed = HMI_Straight_Speed_H<<8+HMI_Straight_Speed_L; //ֱ���ٶ�
			
			HMI_Correction_Speed = *(data+3); //ֱ�������ٶ�
			HMI_Turn_90_Speed = *(data+4); //����ת90���ٶ�
			
			/*
				ֻ��HMI���������ٶȴ���0������Ч���������Ĭ��ֵ
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
	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE); //�����������USART_IT_RXNE���ж��ź� �ŵ���
	//USART_ITConfig(USART3,USART_IT_RXNE,ENABLE); //�����������USART_IT_RXNE���ж��ź� RFID
	USART_ITConfig(UART5, USART_IT_RXNE,ENABLE);  //�����������USART_IT_RXNE���ж��ź� ������
	
	////��־λ��ԭ
	Barrier	= 0; 		//�ϰ����־λ
	Guidance_State = 0;	//�ŵ�����־λ



}


/*********************************************************************************************************
*	�� �� ��: Discern_IC_Card
*	����˵��: �жϵ�ǰ�������Ƿ�ΪĿ�ĵ� ��IC��
*	�� �� ֵ: ��   
*********************************************************************************************************/
void Discern_IC_Card(void)	
{
	/*
	�ֲ�·
	*/
	
	while(HMI_Command[PLACE_Gone]==0x00)	//�޳���ȥ��Ŀ�ĵ�
	{
		PLACE_Gone++;
	}
	
	/*
		����Ŀ�ĵ�
	*/
	Place = HMI_Command[PLACE_Gone];
	/*
	
	*/
	if(Verify_RFID(&RFID_IN[0],&RFID_Fork[0])) 
	{
		/*
		����Ŀ�ĵأ�����Ŀ�ĵأ������ֲ�·ѡ��
		�ڷֲ�·�������ֲ�·�Ŀ�����ʼ����
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
//		ͣ�������ǰ������
//		*/
//		Stop();
//		Direction = Forward;
//		Guidance_State = Auto_Stop;


		
		/*
		 ǰ������ʹ��PE1�ⲿ�жϣ������������� 1~4��̽ͷ
		*/

		
//		if(state ==1)
//		{
//			state = 0;
//			vTaskSuspend(xHandle_BackUp_Barrier);
//			vTaskResume(xHandle_Forward_Barrier);   
//		}

		/*
		 ǰ�����򴮿�2�ĴŴ��������е���
		*/	
//		vTaskSuspend(xHandle_U5_Navigation);  //���𴮿�5 �ŵ���
//		USART_ITConfig(UART5,USART_IT_RXNE,DISABLE);//�رմ���5�ж�
//		vTaskResume(xHandle_U2_Navigation);   //�ָ�����2 �ŵ���
//		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//�򿪴���2�ж�

//		bsp_DelayMS(1000);
//	
//	}
//	else if(Verify_RFID(&RFID_IN[0],&RFID_Back_Up[0])) 
//	{
//		/*
//		ͣ������ɺ���
//		*/
//		
//		Stop();
//		Direction = Back_Up;
//		Guidance_State = Auto_Stop;
		
//		PE1_IN2_NVIC(CLOSE);
//		PE2_IN3_NVIC(OPEN);
		/*
		 ���˷���ʹ��PE2�ⲿ�жϣ������������� 5~8��̽ͷ
		*/
//		state =1;
//		vTaskSuspend(xHandle_Forward_Barrier);   
//		vTaskResume(xHandle_BackUp_Barrier);
//		
		/*
		 ���˷���ʹ�ô���5�ĴŴ��������е���
		*/
//		vTaskResume(xHandle_U2_Navigation);   //�ָ�����2 �ŵ���
//		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//�򿪴���2�ж�
		
//		vTaskSuspend(xHandle_U2_Navigation);  //���𴮿�2 �ŵ���
//		USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);//�رմ���2�ж�
//		vTaskResume(xHandle_U5_Navigation);   //�ָ�����5 �ŵ���
//		USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);//�򿪴���5�ж�
//		//vTaskDelay(1000);
//		bsp_DelayMS(1000);
//	
//	}
//	
	
//	else if(Verify_RFID(&RFID_IN[0],&RFID_90Left[0])) 
//	{
//		/*
//		ͣ����Ȼ�����������ת���ָ���ת90�ȼ������
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
//			����ת��ʱ��Ϊ Turn_Time
//			*/
//			bsp_DelayMS(Turn_Time);
//			
//			/*
//			�����Ѿ�ת��90��
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
//		ͣ����Ȼ�����������ת���ָ���ת90�ȼ������
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
//			����ת��ʱ��Ϊ Turn_Time
//			*/
//			bsp_DelayMS(Turn_Time);
//			
//			/*
//			�����Ѿ�ת��90��
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
	
	
	
	
	
	while(HMI_Command[PLACE_Gone]==0x00)	//�޳���ȥ��Ŀ�ĵ�
	{
		PLACE_Gone++;
	}
	
	/*
		����Ŀ�ĵ�
	*/
	Place = HMI_Command[PLACE_Gone];

	switch(HMI_Command[PLACE_Gone])					
	{			

		//printf("Ŀ�ĵ�%X\r\n",HMI_Command[PLACE_Gone+3]);
		case 0xFA://���
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_START[0])) //�Ƚ�RFID��������RFID_START��1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xFB://�յ�
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_END[0])) //�Ƚ�RFID��������RFID_END��1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xA1: //Ŀ�ĵ�A1
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A1[0])) //�Ƚ�RFID��������RFID_A1��1��ʾ�ص���ϣ�0��ʾ������
			{
				
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬
		
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}

		case 0xA2://Ŀ�ĵ�A2
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A2[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xA3://Ŀ�ĵ�A3
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A3[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);		
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xA4://Ŀ�ĵ�A4
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A4[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}			
		case 0xA5://Ŀ�ĵ�A5
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_A5[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB1://Ŀ�ĵ�B1
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B1[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬		
			}

			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB2://Ŀ�ĵ�B2
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B2[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB3://Ŀ�ĵ�B3
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B3[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB4://Ŀ�ĵ�B4
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B4[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xB5://Ŀ�ĵ�B5
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_B5[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬		
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xC1://Ŀ�ĵ�C1
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C1[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xC2://Ŀ�ĵ�C2
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C2[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xC3://Ŀ�ĵ�C3
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C3[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xC4://Ŀ�ĵ�C4
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C4[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬
					
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}	
		case 0xC5://Ŀ�ĵ�C5
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_C5[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}	
		
		case 0xD1://Ŀ�ĵ�D1
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D1[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);	
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xD2://Ŀ�ĵ�D2
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D2[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬	
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xD3://Ŀ�ĵ�D3
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D3[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}
		case 0xD4://Ŀ�ĵ�D4
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D4[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬
			}
			else
			{
				Place_Stop=0;
			}
			break;
		}	
		case 0xD5://Ŀ�ĵ�D5
		{
			if(Verify_RFID(&RFID_IN[0],&RFID_D5[0])) //1��ʾ�ص���ϣ�0��ʾ������
			{
				RFID_TO_HMI(HMI_Command[PLACE_Gone]);
				State = Parking;   //վ��ͣ��״̬
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
* Description    : ���ƶ���ֱ��
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/

void Go_Straight(void) //ֱ��
{
//	GPIO_ResetBits(GPIOD,GPIO_Pin_14); //�رշ�����
//	GPIO_SetBits(GPIOD,GPIO_Pin_15);// PD15 ǰ���̵���
//	GPIO_ResetBits(GPIOC,GPIO_Pin_8);// PC8	��ת�̵���
//	GPIO_ResetBits(GPIOC,GPIO_Pin_9);// PC9  ��ת�̵���
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);// PD13 ���˼̵���
//	//GPIO_SetBits(GPIOD,GPIO_Pin_15);// PD15 ǰ���̵���	
	//1.д ��������ŷ��� ��ַ1�������� 06 ��ַ 02 ���Ŀ���ٶȣ� 2000 

//	Stop_change = 0;
//	if(Left_change!=Left||Right_change!=Right||Stop_Flag == 1) //��ֹ�ظ����ٶ�ָ��
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
* Description    : ���ƶ��ֺ���
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/



 /*******************************************************************************
* Function Name  : void Turn_Left(void)
* Description    : ���ƶ�����ת
* Input          : None	
* Output 		 : None
* Return         : None
������ �ٶ�-  ��ǰ
������ �ٶ�+  ��ǰ
*******************************************************************************/
void Turn_Left(uint8_t* data) //��ת
{
//	GPIO_ResetBits(GPIOC,GPIO_Pin_8);// PC8	��ת�̵���
//	GPIO_SetBits(GPIOC,GPIO_Pin_9);// PC9  ��ת�̵���
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);// PD13 ���˼̵���
//	GPIO_SetBits(GPIOD,GPIO_Pin_15);// PD15 ǰ���̵���
	//MODH_WriteParam_10H(2,0x0c,2,init_buf); 
	uint16_t Speed_Difference_Correction;
	Stop_change = 0;
	
//	if(Left_change!=Left||Right_change!=Right||Stop_Flag == 1) //��ֹ�ظ����ٶ�ָ��
//	{
	/*
	*data       1 2 3 4 5 6 7 8
	*(data+1)   9 10 11 12 13 14 15 16
	*/
	//ֻ�е��ϴ��Ĵŵ������ݸ���һ�εĲ�һ�������ٴη��źŸ����
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
* Description    : ���ƶ�����ת
* Input          : None	
* Output 		 : None
* Return         : None
������ �ٶ�-  ��ǰ
������ �ٶ�+  ��ǰ
*******************************************************************************/
void Turn_Right(uint8_t* data)//��ת
{
//	GPIO_SetBits(GPIOC,GPIO_Pin_8);// PC8	��ת�̵���
//	GPIO_ResetBits(GPIOC,GPIO_Pin_9);// PC9  ��ת�̵���
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);// PD13 ���˼̵���
	Stop_change = 0;
	uint16_t Speed_Difference_Correction;	
//	if(Left_change!=Left||Right_change!=Right||Stop_Flag == 1) //��ֹ�ظ����ٶ�ָ��
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
* Description    : ���ƶ���ֹͣ
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void Stop(void)//ͣ��
{
//	GPIO_SetBits(GPIOD,GPIO_Pin_14); //�򿪷�����
//	GPIO_ResetBits(GPIOC,GPIO_Pin_8);//��0 PC8	��ת�̵����Ͽ�
//	GPIO_ResetBits(GPIOC,GPIO_Pin_9);//��0 PC9  ��ת�̵����Ͽ�
//	GPIO_ResetBits(GPIOD,GPIO_Pin_13);// PD13 ���˼̵���
//	GPIO_ResetBits(GPIOD,GPIO_Pin_15);//��0 PD15 ǰ���̵����Ͽ�
	
	//MODH_WriteParam_06H(1,2,0);		//�����ŷ����ٶ�ģʽ��Ŀ���ٶ�д0
	Stop_Flag = 1;
	//�������
//	if(Left_change!=Left||Right_change!=Right||Stop_change !=Stop_Flag) //��ֹ�ظ����ٶ�ָ��
//	{

		if(MODH_WriteParam_06H(1,2,0))//�����ŷ����ٶ�ģʽ��Ŀ���ٶ�д0������ͣ��
		{

		}
//		
		if(MODH_WriteParam_06H(2,2,0))//�����ŷ����ٶ�ģʽ��Ŀ���ٶ�д0������ͣ��
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
* Description    : AGV����
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void Alarm(void)//����
{
	//GPIO_SetBits(GPIOD,GPIO_Pin_14); //�򿪷�����
}


 /*******************************************************************************
* Function Name  : void HMI_END(void)
* Description    : ���ͽ�������HMI
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
* Description    : RFID��ʼ����⺯�����ж�RFID�������Ƿ�����
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void RFID_Check(void)
{
	

	/*
	U3_send_buf ��RFID���͵����ݣ�����ģʽ����Ϊ�Զ��������ݣ����Ϊ1�������ϴ�ģʽΪ�Զ��ϴ�ģʽ
	U3_Respond_buf ΪRFID���������óɹ��󷵻ص�����
	*/
	uint8_t U3_send_buf[8]    = {0x3,0x8,0xC1,0x20,0x3,0x1,0x0,0x17}; 
	uint8_t U3_Respond_buf[8] = {0x3,0x8,0xC1,0x20,0x0,0x0,0x0,0x15};
	
	uint8_t U3_si;
	uint8_t U3_check_read;
	uint8_t RFID_Correct = 0;
	uint8_t U3_check_status = 0;
	uint8_t U3_First_Check = 1;
////���ù���ģʽ  �Զ���ȡ������   ��1  �Զ��ϴ�����
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
			printf("���Դ���.��ʾ��.txt=\"RFID��������ʼ����\"");
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
	printf("���Դ���.��ʾ��.txt=\"RFID��������ʼ���ɹ�\"");
	bsp_DelayMS(300);
	HMI_END();
	bsp_DelayMS(1000);
		
}

// /*******************************************************************************
//* Function Name  : void Attitude_Check(void)
//* Description    : ��������������ʼ����⺯�����жϳ������������Ƿ�����
//* Input          : None	
//* Output 		 : None
//* Return         : None
//*******************************************************************************/

//void Attitude_Check(void) //��̬���������
//{
//	uint8_t U5_Attitude_Checked = 0;
//	while(Attitude_Correct!=1)
//	{
//		U5_State( ); //״̬��
//		
//		//��̬���������
//		if(!U5_Attitude_Checked)
//		{
//			printf("���Դ���.��ʾ��.txt=\"��̬������ƫ���ǹ�����\"");
//			bsp_DelayMS(300);
//			HMI_END();
//			bsp_DelayMS(100);

//		}
//		U5_Attitude_Checked =1; //Ϊ��ֻ��HMI��һ�����ݣ����־λ
//	}
//	printf("���Դ���.��ʾ��.txt=\"��̬������ƫ���ǹ���ɹ�\"");
//	bsp_DelayMS(300);
//	HMI_END();
//	bsp_DelayMS(1000);
//	

//}
 /*******************************************************************************
* Function Name  : void U2_Navigation_Check(void)
* Description    : �ŵ�����������ʼ����⺯�����жϴŵ����������Ƿ�����
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void U2_Navigation_Check(void) //�ŵ������
{


	while(U2_Navigation_Correct!=1)
	{
		U2_State(); //״̬��

		if(!U2_Navigation_Checked)
		{
			printf("���Դ���.��ʾ��.txt=\"����2 �ŵ�����������ʼ����\"");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);

		}
		U2_Navigation_Checked =1;
	}
	printf("���Դ���.��ʾ��.txt=\"����2 �ŵ�����������ʼ���ɹ�\"");
	bsp_DelayMS(300);
	HMI_END();
	bsp_DelayMS(1000);
	U2_ucCount = 0;

}


 /*******************************************************************************
* Function Name  : void U5_Navigation_Check(void)
* Description    : �ŵ�����������ʼ����⺯�����жϴŵ����������Ƿ�����
* Input          : None	
* Output 		 : None
* Return         : None
*******************************************************************************/
void U5_Navigation_Check(void) //�ŵ������
{

	//uint8_t U2_Navigation_Checked = 0;
	while(U5_Navigation_Correct!=1)
	{
		U5_State(); //״̬��

		if(!U5_Navigation_Checked)
		{
			printf("���Դ���.��ʾ��.txt=\"����5 �ŵ�����������ʼ����\"");
			bsp_DelayMS(300);
			HMI_END();
			bsp_DelayMS(100);

		}
		U5_Navigation_Checked =1;
	}
	printf("���Դ���.��ʾ��.txt=\"����5 �ŵ�����������ʼ���ɹ�\"");
	bsp_DelayMS(300);
	HMI_END();
	bsp_DelayMS(1000);
	U5_ucCount = 0;

}



 /*******************************************************************************
* Function Name  : void RFID_TO_HMI(void)
* Description    : �����ж�ΪĿ�ĵ�֮�󣬷��͵�ǰͣ�����HMI������
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
* Description    : ͨ�����緢��AGV״̬����
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_To_Network(uint8_t *_Work_Mode,uint8_t *_Reserved)    //ͨ�����緢��AGV״̬����
{

	int i;
	
	uint8_t testbuf[1] = {0x0a};
	/*
	���Ͱ�ͷ
	*/
//	Network_Reserved_BUF[0] = Network_Head_BUF[0];
//	Network_Reserved_BUF[1] = Network_Head_BUF[1];
	for(i=0;i<2;i++)
	{
		Network_AGV_BUF[i] = Network_Head_BUF[i];
	}
	
	
	
	/*
	AGV�豸��ʶ
	*/	
	for(i=0;i<3;i++)
	{
		Network_AGV_BUF[i+2] = Network_AGVID_BUF[i];
	}


	
	
	/*
	AGV����ģʽ
	*/
	Network_AGV_BUF[5] = *_Work_Mode;

	
	
	/*
	 AGV״̬ + ǰ���ص�
	*/	
	Network_AGV_BUF[6] = State;
	Network_AGV_BUF[7] = Place;
	
	/*
	 8��Ԥ�� �ֽ�
	*/	
	
	
	//Network_Reserved_Send(_Reserved);
	
	
	
	
	/*
	���Ͱ�β
	*/	
	Network_AGV_BUF[16] = Network_End_BUF[0];
	
	comSendBuf(COM4, (uint8_t *)Network_AGV_BUF, sizeof(Network_AGV_BUF));
	comSendBuf(COM4, (uint8_t *)testbuf, sizeof(testbuf));
	
}


 /*******************************************************************************
* Function Name  : void Network_Head_Send(void)
* Description    : ����ͨѶЭ���ͷ
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
* Description    : ���� AGV ID ÿ̨AGV�ж�����ID����ҪԤ��
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
* Description    : ���� AGV ��ǰ״̬
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void Network_AGVState_Send(uint8_t *_State,uint8_t *_Place) 
{
	uint8_t *None = 0;
	comSendBuf(COM4, (uint8_t *)_State, Network_State_LEN);
	/*
		_State == 0xC1��ʾ����
		Ŀǰ�����޷��жϵص㣬������������£�ֻ�е��ص�����ʱ��Ŵ��ڴ������
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
* Description    : Ԥ����8���ֽڿռ䣬��������չ
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
* Description    : ����Э���β
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
* Description    : �ָ�������
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
* Description    : ��������ʱ�̵���
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/

void Running_Green_Light(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_15); 	//G
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);	//Y			
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);	//R	
	//printf("�̵���\r\n");

}

 /*******************************************************************************
* Function Name  : void Waiting_Yellow_Light(void)
* Description    : վ��ͣ��ʱ�Ƶ���
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/

void Waiting_Yellow_Light(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_15); 	//G
	GPIO_SetBits(GPIOC,GPIO_Pin_9);	 	//Y			
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);	//R	
	//printf("�Ƶ���\r\n");

}

 /*******************************************************************************
* Function Name  : void Alarm_Red_Light(void)
* Description    : վ��ͣ��ʱ�Ƶ���
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/

void Alarm_Red_Light(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_15); 	//G
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);	//Y			
	GPIO_SetBits(GPIOC,GPIO_Pin_8);		//R	
	//printf("�����\r\n");

}

 /*******************************************************************************
* Function Name  : void AGV_Have_Barrier(void)
* Description    :  AGVǰ�����ϰ��AGVͣ�����������Ϲ��� vTaskSuspend(xHandle_AGV_Action);
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_Have_Barrier(void)
{
	Stop();
	/*
	 ����AGV����
	*/
	vTaskSuspend(xHandle_AGV_Action);
	vTaskSuspend(xHandle_U2_Navigation);
	vTaskSuspend(xHandle_U5_Navigation);
	/*
	  ����ʱ���ɫ����
	*/
	Alarm_Red_Light();	
	
	State = Barrier_ERR  ; //�ϰ���״̬
	printf("��ʻ����.�ϰ���.val=1");
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(1000);
	
	
}
 /*******************************************************************************
* Function Name  : void AGV_None_Barrier(void)
* Description    : AGVǰ��û���ϰ���ָ�����vTaskResume(xHandle_AGV_Action);
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_None_Barrier(void)
{	
	State = Moving  ; 
	Barrier = 0;
	Guidance_State = Auto_Wait;
	printf("��ʻ����.�ϰ���.val=0");
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(1000);
	vTaskResume(xHandle_U2_Navigation);
	vTaskResume(xHandle_U5_Navigation);
	vTaskResume(xHandle_AGV_Action);

}



 /*******************************************************************************
* Function Name  : void AGV_StopBManual(void)
* Description    : �ֶ�ֹͣ
* Input          : 
* Output 		 : None
* Return         : None
*******************************************************************************/
void AGV_StopBManual(void)
{
	Stop();
	/*
	 ����AGV����
	*/
	vTaskSuspend(xHandle_AGV_Action);
	vTaskSuspend(xHandle_U2_Navigation);
	vTaskSuspend(xHandle_U5_Navigation);
	
	State = Manual_Stop  ; //�ֶ�ֹͣ
	/*
	  ����ʱ���ɫ����
	*/
	Waiting_Yellow_Light();	

}
 /*******************************************************************************
* Function Name  : void AGV_RunBManual(void)
* Description    : �ֶ��ָ�
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
* Description    : �ж�AGV�Ƿ��Ѿ���ת90��   ����U2�ŵ���������
* Input          : 
* Output 		 : None
* Return         : None

	
		*data 		01~08
		*(data+1)	09~16
	
		��01~05̽ͷ����������ת
		
		��12~16̽ͷ����������ת
	
*******************************************************************************/
void U2_Left_90Angel_Check(void)
{	
	/*
		�жϷ�����
		1. 5 6 7 8 9 10 11 12̽ͷ�ζ�û��Ӧ���źţ���Ϊ�ǿ�ʼ����
		//2. 5 6 7 8 9 10 11 12̽ͷ��һһ������������Ϊ�ǽ�������
		2. 7 8 9 10 ̽ͷ��һһ������������Ϊ�ǽ�������
	*/

	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //�������ݵİ�ͷ 5���ֽ�    ʮ��·�ŵ���
//	uint8_t Navigation_9_10 ; //9 10��̽ͷ
//	uint8_t Navigation_7_8 ; //7 8��̽ͷ
	uint8_t Navigation_10_11_12_13; //10 11 12 13
	uint8_t ALL_1_8;
	uint8_t	ALL_9_16;
	uint8_t Left_90Angel_Check;
	if (comGetChar(COM2, &U2_read))
	{
		switch (U2_ucStatus)
		{
			/* ״̬0��֤���յ�0x01 */
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
				if(U2_read!=0x55) ///�����и�BUG,��һ������Ϊ0X55
				{
					U2_buf[U2_ucCount] = U2_read;              
					 /* ���չ�2������ */
					if(U2_ucCount == 2-1)
					{	
						//printf("�����ݽ���\r\n");
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
	//ֻ�е� Left_90Angel_Check = 1 ��ִ���������
	//���ŵ����ϴ������ݰ�ͷ��β����ȷ��Left_90Angel_Check�ŵ���1
	if(Left_90Angel_Check ==1 )
	{
		//Left_90Angel_Check = 0;
		switch (State_16)
		{
			case 0:
			{
				if((ALL_1_8 ==0) &&(ALL_9_16 ==0)) //16��Ϊ0
				{
					State_16 = 1;   
				}
						
				break;
			}
			
			case 1:
			{
				//if((Navigation_7_8 == 0x03)&& (Navigation_9_10 == 0xC0)) // 7 8 9 10 ͬʱ��⵽����
				if(Navigation_10_11_12_13) //10 11 12 13 ͬʱ��⵽����
				{
					State_16 = 0;   
					/*
						�Ѿ���ת90��
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
* Description    : �ж�AGV�Ƿ��Ѿ���ת90�� ����U2 �ŵ���
* Input          : 
* Output 		 : None
* Return         : None




*******************************************************************************/
void U2_Right_90Angel_Check(void)
{	
	/*
		�жϷ�����
		1. ����̽ͷ�ζ�û��Ӧ���źţ���Ϊ�ǿ�ʼ����
		2. 5 6 7 8 9 10 11 12̽ͷ��һһ������������Ϊ�ǽ�������
	*/

	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //�������ݵİ�ͷ 5���ֽ�    ʮ��·�ŵ���
//	uint8_t Navigation_9_10 ; //9 10��̽ͷ
//	uint8_t Navigation_7_8 ; //7 8��̽ͷ
	//uint8_t Navigation_4567;
	uint8_t Navigation_5678;
	uint8_t ALL_1_8;
	uint8_t	ALL_9_16;
	uint8_t Right_90Angel_Check;
	if (comGetChar(COM2, &U2_read))
	{
		switch (U2_ucStatus)
		{
			/* ״̬0��֤���յ�0x01 */
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
	//ֻ�е� Right_90Angel_Check = 1 ��ִ���������
	//���ŵ����ϴ������ݰ�ͷ��β����ȷ��Right_90Angel_Check�ŵ���1
	if(Right_90Angel_Check ==1 )
	{
		Right_90Angel_Check = 0;
		switch (State_16)
		{
			case 0:
			{
				if((ALL_1_8 ==0) &&(ALL_9_16 ==0)) // 1.���뿪����
				{
					State_16 = 1;   
				}
						
				break;
			}
			
			case 1:
			{
				//if((Navigation_7_8 == 0x03)&& (Navigation_9_10 == 0xC0)) // 7 8 9 10 ͬʱ��⵽����
				if(Navigation_5678 == 0x0F) // 4 5 6 7 ͬʱ��⵽����
				{
					State_16 = 0;   
					/*
						�Ѿ���ת90��
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
* Description    : �ж�AGV�Ƿ��Ѿ���ת90��   ����U5�ŵ���������
* Input          : 
* Output 		 : None
* Return         : None

	
		*data 		01~08
		*(data+1)	09~16
	
		��01~05̽ͷ����������ת
		
		��12~16̽ͷ����������ת
	
*******************************************************************************/
void U5_Left_90Angel_Check(void)
{	
//	/*
//		�жϷ�����
//		����Ϊʮ���Σ�����һ�������뿪����ת�Ƶ����ڵĴ����ϣ������Ϊ90��
//		��01��̽ͷ��0��1�ٵ�0���ж�Ϊ�뿪�˵�ǰ����
//		��07��̽ͷΪ1�����ж�Ϊ�Ѿ�ת�Ƶ��µĴ�����
//	*/

	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //�������ݵİ�ͷ 5���ֽ�    ʮ��·�ŵ���
	uint8_t Navigation_9_10 ; //16��̽ͷ
	uint8_t Navigation_7_8 ; //10��̽ͷ
	uint8_t ALL_1_8;
	uint8_t	ALL_9_16;
	uint8_t Left_90Angel_Check;
	if (comGetChar(COM5, &U5_read))
	{
		switch (U5_ucStatus)
		{
			/* ״̬0��֤���յ�0x01 */
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
				if(U5_read!=0x55) ///�����и�BUG,��һ������Ϊ0X55
				{
					U5_buf[U5_ucCount] = U5_read;              
					 /* ���չ�2������ */
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
	//ֻ�е� Left_90Angel_Check = 1 ��ִ���������
	//���ŵ����ϴ������ݰ�ͷ��β����ȷ��Left_90Angel_Check�ŵ���1
	if(Left_90Angel_Check ==1 )
	{
		Left_90Angel_Check = 0;
		switch (State_16)
		{
			case 0:
			{
				if((ALL_1_8 ==0) &&(ALL_9_16 ==0)) //16��Ϊ0
				{
					State_16 = 1;   
				}
						
				break;
			}
			
			case 1:
			{
				if((Navigation_7_8 == 0x03)&& (Navigation_9_10 == 0xC0)) // 7 8 9 10 ͬʱ��⵽����
				{
					State_16 = 0;   
					/*
						�Ѿ���ת90��
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
* Description    : �ж�AGV�Ƿ��Ѿ���ת90�� ����U5 �ŵ���
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void U5_Right_90Angel_Check(void)
{	
	/*
		�жϷ�����
		����Ϊʮ���Σ�����һ�������뿪����ת�Ƶ����ڵĴ����ϣ������Ϊ90��
		��16��̽ͷ��0��1�ٵ�0���ж�Ϊ�뿪�˵�ǰ����
		��10��̽ͷΪ1�����ж�Ϊ�Ѿ�ת�Ƶ��µĴ�����
	*/

	const uint8_t CDH_HEAD[2]= {0x55,0x54}; //�������ݵİ�ͷ 5���ֽ�    ʮ��·�ŵ���
	uint8_t Navigation_9_10 ; //16��̽ͷ
	uint8_t Navigation_7_8 ; //10��̽ͷ
	uint8_t ALL_1_8;
	uint8_t	ALL_9_16;
	uint8_t Right_90Angel_Check;
	if (comGetChar(COM5, &U5_read))
	{
		switch (U5_ucStatus)
		{
			/* ״̬0��֤���յ�0x01 */
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
				if(U5_read!=0x55) ///�����и�BUG,��һ������Ϊ0X55
				{
					U5_buf[U5_ucCount] = U5_read;              
					 /* ���չ�2������ */
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
	//ֻ�е� Right_90Angel_Check = 1 ��ִ���������
	//���ŵ����ϴ������ݰ�ͷ��β����ȷ��Right_90Angel_Check�ŵ���1
	if(Right_90Angel_Check ==1 )
	{
		Right_90Angel_Check = 0;
		switch (State_16)
		{
			case 0:
			{
				if((ALL_1_8 ==0) && (ALL_9_16 ==0)) // 1.���뿪����
				{
					State_16 = 1;   
				}
						
				break;
			}
			
			case 1:
			{
				if((Navigation_7_8 == 0x03)&& (Navigation_9_10 == 0xC0)) // 7 8 9 10 ͬʱ��⵽����
				{
					State_16 = 0;   
					/*
						�Ѿ���ת90��
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
* Description    : ������ת90��
* Input          : 
* Output 		 : None
* Return         : None
������ �ٶ�-  ��ǰ
������ �ٶ�+  ��ǰ
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
* Description    : ������ת90��
* Input          : 
* Output 		 : None
* Return         : None
������ �ٶ�-  ��ǰ
������ �ٶ�+  ��ǰ
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
* Description    : ֱ��
* Input          : 
* Output 		 : None
* Return         : None
������ �ٶ�-  ��ǰ
������ �ٶ�+  ��ǰ
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
* Description    : ����
* Input          : 
* Output 		 : None
* Return         : None
������ �ٶ�-  ��ǰ
������ �ٶ�+  ��ǰ

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

TBD:�ݶ������ж�ת���Ƿ�ʱ
��10���ڻ�û��⵽���������ж�ת�䳬ʱ
*/
void Time_Out(void)
{
	//int32_t turn_time;
	//turn_time = bsp_GetRunTime();	/* ��¼����͵�ʱ�� */
//	if (bsp_CheckRunTime(turn_time) > TIMEOUT_Turn)	
//	{
//	
//	}

}


 /*******************************************************************************
* Function Name  : void Path_Choice(void)
* Description    : ·��ѡ��
* Input          : Ŀ�ĵ� uint8_t Place;
* Output 		 : ֱ�С���ת����ת�ź�
* Return         : �ֲ�··��ѡ���־λ
����ȡ���ֲ��2��ʱ����Ҫ�ж�Ŀǰ��Ҫǰ����Ŀ�ĵ�λ���ĸ�����

ֱ�У�Ŀ�ĵ���C�� ֱ�ж�
��ת��Ŀ�ĵ���B�� ��ת��
��ת��Ŀ�ĵ���D�� ��ת��

�ѵص��Ϊ�Σ��൱�ڼ���
ͨ���������飬�ж�Ŀ�ĵ������ĸ�����
uint8_t Straight_Sets[10]   = {0xC1,0xC2,0xC3,0xC4,0xC5};
uint8_t Left_Turn_Sets[10]  = {0xB1,0xB2,0xB3,0xB4,0xB5};
uint8_t Right_Turn_Sets[10] = {0xD1,0xD2,0xD3,0xD4,0xD5};
uint8_t Back_Turn_Sets[10] = {0xA1,0xA2,0xA3,0xA4,0xA5};

�ֲ��ѡ���־λ

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
	�ж��Ƿ�����C�� ֱ�ж�
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
* Description    : �ֲ�·��ת
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
//		//printf("����ѭ��");
//		Read_Speed();
//		Send_Speed();
//	}
//	/*
//	����ת��ʱ��Ϊ Turn_Time
//	*/
//	while(time_count)
//	{
//		bsp_DelayMS(1000);
//		time_count = time_count - 1;
//	}

//	
//	/*
//	�����Ѿ�ת��90��
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
* Description    : �ֲ�·��ת
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
//	�����Ѿ�ת��90��
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
* Description    : �ֲ�·ֱ��
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
//			��ִֵ������ٽ���buff_count++��buff_val++
//		*/
////		test_buff[buff_count++] = buff_val++;
////		printf("����Ϊ%d",test_buff[buff_count-1]);
////		printf("���Ϊ%d",buff_count);
////		printf("����Ϊ%d",buff_val);
//		
//		//buff_count++;
//		bsp_DelayMS(500);
//		//printf("����ѭ��");	
//		Read_Speed();
//		Send_Speed();
//	}

}
/*******************************************************************************
* Function Name  : void Fork_Back(void)
* Description    : �ֲ�·ֱ��
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
//			��ִֵ������ٽ���buff_count++��buff_val++
//		*/
////		test_buff[buff_count++] = buff_val++;
////		printf("����Ϊ%d",test_buff[buff_count-1]);
////		printf("���Ϊ%d",buff_count);
////		printf("����Ϊ%d",buff_val);
//		
//		//buff_count++;
//		bsp_DelayMS(500);
//		//printf("����ѭ��");	
//		Read_Speed();
//		Send_Speed();
//	}

}

/*******************************************************************************
* Function Name  : void Read_Speed(void)
* Description    : ��ȡ����ٶ�
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void Read_Speed(void)
{
	//uint8_t MODH_ReadParam_03H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num)
	
	//��ȡ1�ŵ���ĵ�ǰ�ٶ�
	//���ʵ��ת�� = ��ȡ�����ٶ�/10
	if(MODH_ReadParam_03H(1,0x10,1))
	{
//		Read_Speed_1_H = g_tVar.P01;
//		Read_Speed_1_L = g_tVar.P02;
//		Read_Speed_1 = (Read_Speed_1_H<<8+Read_Speed_1_L)/10;
		//printf("�����ٶ�=%d",g_tVar.P01);
		if(g_tVar.P01>32767)
		{
			g_tVar.P01 = ((65535-g_tVar.P01)+1);
		}	
		//Read_Speed_1 = (g_tVar.P01)/10;
		Read_Speed_1 = (g_tVar.P01);
	}
	else
	{
		//printf("��ȡ�����ٶ�ʧ��");

	}
	//��ȡ2�ŵ���ĵ�ǰ�ٶ�
	//���ʵ��ת�� = ��ȡ�����ٶ�/10
	if(MODH_ReadParam_03H(2,0x10,1))
	{
//		Read_Speed_2_H = g_tVar.P01;;
//		Read_Speed_2_L = g_tVar.P02;
//		Read_Speed_2 = (Read_Speed_2_H<<8+Read_Speed_2_L)/10;
		//printf("�����ٶ�=%d",g_tVar.P01);
		if(g_tVar.P01>32767)
		{
			g_tVar.P01 = ((65535-g_tVar.P01)+1);
		}	
		//Read_Speed_2 = (g_tVar.P01)/10;
		Read_Speed_2 = (g_tVar.P01);
	}
	else
	{
		//printf("��ȡ�����ٶ�ʧ��");

	}
	

}
/*******************************************************************************
* Function Name  : void Read_Speed(void)
* Description    : ��ȡ����ٶ�
* Input          : 
* Output 		 : None
* Return         : None

*******************************************************************************/
void Send_Speed(void)
{
	//printf("��ȡ���=%X\r\n",g_tVar.P01);
	//ת����ʾ.����ת��.val=\"%d\""
	printf("ת����ʾ.����ת��.val=%d",Read_Speed_1);
	bsp_DelayMS(100);
	HMI_END();
	bsp_DelayMS(100);
	
	printf("ת����ʾ.����ת��.val=%d",Read_Speed_2);
	bsp_DelayMS(100);
	HMI_END();
	
//	printf("ת����ʾ.����ת��.val=%d\r\n",Read_Speed_1);
//	printf("ת����ʾ.����ת��.val=%d\r\n",Read_Speed_2);

}



