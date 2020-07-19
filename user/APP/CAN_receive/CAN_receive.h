/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{



	CAN_CHASSIS_ALL_ID = 0x200,	//CAN发送报文标识符
    CAN_3508_M1_ID = 0x201,			//CAN反馈报文标识符
    CAN_3508_M2_ID = 0x202,			//CAN反馈报文标识符
    CAN_3508_M3_ID = 0x203,			//CAN反馈报文标识符
    CAN_3508_M4_ID = 0x204,			//CAN反馈报文标识符

    CAN_YAW_MOTOR_ID = 0x205,		//CAN反馈报文标识符
    CAN_PIT_MOTOR_ID = 0x206,		//CAN反馈报文标识符
    CAN_SHOOT_ID = 0x207,//CAN反馈报文标识符
		CAN_PUSH_ID = 0x208,
		CAN_GIMBAL_ALL_ID = 0x1FF,	//CAN发送报文标识符
	
		CAN_SetSuperPower_ID = 0x301,
		CAN_SuperPower_MCU_ID = 0x302,
		

} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;								//转子机械角度  0-8191对应0-360度
    int16_t speed_rpm;					//转子转速 单位：RPM
    int16_t given_current;			//控制电流 -16384-0-16384对应-20-0-20A
    uint8_t temperate;					//电机温度 单位：℃
    int16_t last_ecd;						//上次转子机械角度
} motor_measure_t;

typedef struct
{
		uint8_t msg1;
		uint8_t msg2;
		uint16_t	msg3;
		uint8_t	msg4;
		uint8_t msg5;
		uint8_t msg6;
		uint8_t msg7;
		uint8_t msg8;
		
} extended_mcu_msg_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//发送云台控制命令，其中rev为保留字节
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void SetSuperPowerMcuMsg(uint8_t msg1 ,uint8_t msg2, uint8_t msg3, uint8_t msg4,uint8_t msg5,uint8_t msg6,uint8_t msg7,uint8_t msg8);

//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回Shoot电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Shoot_Motor_Measure_Point(void);
//返回Push电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Push_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

extern const extended_mcu_msg_t *get_SuperPowerMcu_Msg_Point(void);


#endif
