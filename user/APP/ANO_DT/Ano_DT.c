/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：数据传输
**********************************************************************************/
#include "Ano_DT.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define MYHWADDR	0x05
#define SWJADDR		0xAF

#define PARNUM		100
s32 ParValList[100];		//参数列表

dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[50];	//发送数据缓存
u8 checkdata_to_send,checksum_to_send;

/*******************/
// 灵活格式帧
// 格式：帧头0xAA 目标地址0xFF 功能码ID 数据长度1-40（单位为字节） 和校验 附加校验
// ID：0xF1 - 0xFA

robot_status_dt_t robot_status_dt_dataset;
u8 DataToSend[100];	//定义数据缓存区

static void ANODT_DATA_Init(robot_status_dt_t *robot_status);

static void ANODT_DATA_Init(robot_status_dt_t *robot_status)
{
		robot_status->power_heat_data_pointer = get_power_heat_data_Point();
		robot_status->robot_state_pointer = get_robot_state_data_Point();
		robot_status->voltage_pointer = get_Voltage_Point();
}


void ANODT_DATA_Setup(void)
{
		ANODT_DATA_Init(&robot_status_dt_dataset);
}

void ANODT_DATA_Task(void)
{
		ANODT_SendF2((u16)((robot_status_dt_dataset.power_heat_data_pointer->chassis_power)*100),
												robot_status_dt_dataset.power_heat_data_pointer->chassis_power_buffer,
												robot_status_dt_dataset.robot_state_pointer->robot_id,
												robot_status_dt_dataset.robot_state_pointer->remain_HP,
												(u16)((*robot_status_dt_dataset.voltage_pointer)*100));
}




void ANODT_SendF1(s16 _a, s16 _b, s32 _c) //0xF1的ID demo
{
	u8 _cnt = 0;
	
	DataToSend[_cnt++] = 0xAA;	//帧头
	DataToSend[_cnt++] = 0xFF;	//目标地址
	DataToSend[_cnt++] = 0xF1;	//CMD_ID
	DataToSend[_cnt++] = 8;	//数据长度，字节为单位
	
	//小端模式，数据低字节保存在内存低地址中，高字节在后
	DataToSend[_cnt++] = BYTE0(_a);
	DataToSend[_cnt++] = BYTE1(_a);
	
	DataToSend[_cnt++] = BYTE0(_b);
	DataToSend[_cnt++] = BYTE1(_b);
	
	DataToSend[_cnt++] = BYTE0(_c);
	DataToSend[_cnt++] = BYTE1(_c);
	DataToSend[_cnt++] = BYTE2(_c);
	DataToSend[_cnt++] = BYTE3(_c);
	
	//计算 和校验 附加校验
	u8 sc = 0;
	u8 ac = 0;
	for(u8 i = 0; i<DataToSend[3]+4; i++)
	{
		sc += DataToSend[i];
		ac += sc;
	}	
	DataToSend[_cnt++] = sc;
	DataToSend[_cnt++] = ac;
	
	UART7_Put_Buf(DataToSend,_cnt);
}

void ANODT_SendF2(u16 _a, u16 _b, u8 _c,u16 _d,u16 _e) //0xF2的ID demo
{
	u8 _cnt = 0;
	
	DataToSend[_cnt++] = 0xAA;	//帧头
	DataToSend[_cnt++] = 0xFF;	//目标地址
	DataToSend[_cnt++] = 0xF2;	//CMD_ID
	DataToSend[_cnt++] = 9;	//数据长度，字节为单位
	
	//小端模式，数据低字节保存在内存低地址中，高字节在后
	DataToSend[_cnt++] = BYTE0(_a);
	DataToSend[_cnt++] = BYTE1(_a);
	
	DataToSend[_cnt++] = BYTE0(_b);
	DataToSend[_cnt++] = BYTE1(_b);
	
	DataToSend[_cnt++] = BYTE0(_c);
	
	DataToSend[_cnt++] = BYTE0(_d);
	DataToSend[_cnt++] = BYTE1(_d);
	
	DataToSend[_cnt++] = BYTE0(_e);
	DataToSend[_cnt++] = BYTE1(_e);

	
	//计算 和校验 附加校验
	u8 sc = 0;
	u8 ac = 0;
	for(u8 i = 0; i<DataToSend[3]+4; i++)
	{
		sc += DataToSend[i];
		ac += sc;
	}	
	DataToSend[_cnt++] = sc;
	DataToSend[_cnt++] = ac;
	
	UART7_Put_Buf(DataToSend,_cnt);
}
/*********************/
// 参数读写返回+匿名安全通信协议
//s32 userPar10 = 123;

//void ANODT_SendPar(u16 _id, s32 _val)
//{
//	u8 _cnt = 0;
//	u8 check_sum1 = 0, check_sum2 = 0;
//	
//	DataToSend[_cnt++] = 0xAA;	//帧头
//	DataToSend[_cnt++] = 0xFF;	//目标地址:广播
//	DataToSend[_cnt++] = 0xE2;	//CMD_ID：参数读写返回
//	DataToSend[_cnt++] = 0;	//数据长度，字节为单位
//	
//	//小端模式，数据低字节保存在内存低地址中，高字节在后
//	DataToSend[_cnt++] = BYTE0(_id);
//	DataToSend[_cnt++] = BYTE1(_id);
//	
//	DataToSend[_cnt++] = BYTE0(_val);
//	DataToSend[_cnt++] = BYTE1(_val);
//	DataToSend[_cnt++] = BYTE2(_val);
//	DataToSend[_cnt++] = BYTE3(_val);
//	
//	DataToSend[3] = _cnt - 4;
//	//计算 和校验 附加校验
//	for(u8 i = 0; i<DataToSend[3]+4; i++)
//	{
//		check_sum1 += DataToSend[i];
//		check_sum2 += check_sum1;
//	}	
//	DataToSend[_cnt++] = check_sum1;
//	DataToSend[_cnt++] = check_sum2;
//	
//	Usart2_Put_Buf(DataToSend,_cnt);
//}
//u8 DataGet[60];

//void ANODT_Anl(void)
//{
//	u8 sc = 0;
//	u8 ac = 0;
//	u8 _datalen = DataGet[3];
//	
//	for(u8 i = 0;i<DataGet[3]+4;i++)
//	{
//		sc += DataGet[i];
//		ac += sc;
//	}
//	//和校验 附加校验判断
//	if(sc != DataGet[DataGet[3]+4] || ac != DataGet[DataGet[3]+5])
//		return;
//	
//	if(DataGet[2] == 0xE1)
//	{
//		//小端模式,读取参数
//		u16 _id = DataGet[4] + (u16)(DataGet[5]<<8);
//		switch(_id)
//		{
//			case 10:
//				ANODT_SendPar(_id, userPar10);	break;
//			default:
//				ANODT_SendPar(_id,	0);	break;
//		}
//	}
//}
//void ANODT_GetByte(u8 data)
//{
//	static u8 _sta = 0;
//	static u8 _datalen = 0;
//	static u8 _datacnt = 0; //已经接收到
//	
//	if(_sta == 0)
//	{
//		DataGet[0] = data;
//		if(data == 0xAA)
//				_sta = 1;
//	}	
//	else if(_sta == 1)
//	{
//		DataGet[1] = data;
//		_sta = 2;
//	}
//	else if(_sta == 2)
//	{
//		DataGet[2] = data;
//		_sta = 3;
//	}
//	else if(_sta == 3)
//	{
//		if(data > 50) //数据长度大于50的话代表读错了
//			_sta = 0;		//重新读取
//		else
//		{
//			_sta = 4;
//			DataGet[3] = data;
//			_datalen = data;
//		}
//	}
//	else if(_sta == 4)//
//	{
//		DataGet[4+_datacnt++] = data;		
//		if(_datacnt >= _datalen)
//			_sta = 5;
//	}
//	else if(_sta == 5) //和校验
//	{
//		DataGet[4+_datacnt] = data;
//		_sta = 6;
//	}
//	else if(_sta ==6) //附加校验
//	{
//		DataGet[4+_datacnt] = data;
//		_sta = 0;
//		ANODT_Anl();
//	}
//}