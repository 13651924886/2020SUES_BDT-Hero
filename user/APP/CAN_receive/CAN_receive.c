/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2019-11-16     	Stone              1. 完成
  *
  @verbatim
  ==============================================================================
	程序流程解释：
	
		电调向CAN1总线发送反馈报文 ---->CAN1_RX0_IRQHandler() ----> CAN_hook() ----> get_motor_measure(ptr, rx_message)
																								 或 get_gimbal_motor_measuer(ptr, rx_message)
																								 
		电调向CAN2总线发送反馈报文 ---->CAN2_RX0_IRQHandler() ----> CAN_hook() ----> get_motor_measure(ptr, rx_message)
																								 或 get_gimbal_motor_measuer(ptr, rx_message)
						反馈信息分别存放在motor_yaw, motor_pit, motor_trigger, motor_chassis[4]结构体变量中
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

/************************* 头文件 ******************************/

#include "CAN_Receive.h"

#include "stm32f4xx.h"
#include "rng.h"

#include "Detect_Task.h"


/*********************** 宏定义函数 ****************************/

//底盘电机数据读取
//在CAN_hook()中使用
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//云台电机数据读取
//CAN_hook()中使用
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
		
#define get_super_power_mcu_msg(ptr, rx_message)																							\
		{																																													\
				(ptr)->msg1 = (rx_message)->Data[0]; 					\
				(ptr)->msg2 = (rx_message)->Data[1];					\
				(ptr)->msg3 = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);					\
				(ptr)->msg4 =	(rx_message)->Data[3];					\
				(ptr)->msg5 =	(rx_message)->Data[4];					\
				(ptr)->msg6 =	(rx_message)->Data[5];					\
				(ptr)->msg7 =	(rx_message)->Data[6];					\
				(ptr)->msg8 =	(rx_message)->Data[7];					\
		}	
/************************ 函数定义 ******************************/
		
//报文解析函数
//CAN1_RX0_IRQHandler()中使用
//CAN2_RX0_IRQHandler()中使用
static void CAN_hook(CanRxMsg *rx_message);
		
		
		
/*********************** 全局变量定义 ***************************/
//声明电机变量
motor_measure_t motor_yaw, motor_pit, motor_push, motor_shoot , motor_chassis[4]; //方便Debug查看变量 把静态去掉
		
extended_mcu_msg_t	super_power_mcu_msg;

static CanTxMsg GIMBAL_TxMessage;
		
		
		
		
/************************ 函数内容 ******************************/
//can1接收中断服务函数
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN_hook(&rx1_message);
    }
}

//can2接收中断服务函数
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN_hook(&rx2_message);
    }
}

//发送云台控制命令，其中rev为保留字节
//Gimbal_Task里用
void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = (rev >> 8);
    GIMBAL_TxMessage.Data[7] = rev;
    CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );


}

void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}
//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
//在任务中没有使用
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN2, &TxMessage);
}

//发送底盘电机控制命令
//在chassis_task中使用
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//返回Shoot电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Shoot_Motor_Measure_Point(void)
{
    return &motor_shoot;
}
//返回Push电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Push_Motor_Measure_Point(void)
{
    return &motor_push;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];//i & 0000 0011
}

const extended_mcu_msg_t *get_SuperPowerMcu_Msg_Point(void)
{
    return &super_power_mcu_msg;//i & 0000 0011
}

void SetSuperPowerMcuMsg(uint8_t msg1 ,uint8_t msg2, uint8_t msg3, uint8_t msg4,uint8_t msg5,uint8_t msg6,uint8_t msg7,uint8_t msg8)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    can_id = CAN_SetSuperPower_ID; 
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = msg1;
    tx_message.Data[1] = msg2;
    tx_message.Data[2] = msg3;
    tx_message.Data[3] = msg4;
    tx_message.Data[4] = msg5;
    tx_message.Data[5] = msg6;
    tx_message.Data[6] = msg7;
    tx_message.Data[7] = msg8;
    
    CAN_Transmit(CAN1,&tx_message);
}


//CAN总线电调反馈报文解析函数，并且记录发送数据的时间，作为离线判断依据
//临时去掉DetectTask任务
static void CAN_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
        //处理电机数据宏函数
        get_gimbal_motor_measuer(&motor_yaw, rx_message);
        //记录时间
        //DetectHook(YawGimbalMotorTOE);  //临时去掉DetectTask任务
        break;
    }
    case CAN_PIT_MOTOR_ID:
    {
        //处理电机数据宏函数
        get_gimbal_motor_measuer(&motor_pit, rx_message);
        //DetectHook(PitchGimbalMotorTOE);
        break;
    }
    case CAN_SHOOT_ID:
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_shoot, rx_message);
        //记录时间
        //DetectHook(TriggerMotorTOE);  //临时去掉DetectTask任务
        break;
    }
		case CAN_PUSH_ID:
		{
			  get_gimbal_motor_measuer(&motor_push, rx_message);
				break;
		}
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //处理电机ID号
        i = rx_message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        get_motor_measure(&motor_chassis[i], rx_message);
        //记录时间
        //DetectHook(ChassisMotor1TOE + i); //临时去掉DetectTask任务
        break;
    }
		case CAN_SuperPower_MCU_ID:
		{
				get_super_power_mcu_msg(&super_power_mcu_msg, rx_message);		
				break;
		}

    default:
    {
        break;
    }
    }
}
