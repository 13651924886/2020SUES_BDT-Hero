/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能，其中射击的初始化，以及循环都是在云台任务中调用，故而此文件
  *             不是freeRTOS任务，射击分关闭状态，准备状态，射击状态，以及完成状态
  *             关闭状态是关闭摩擦轮以及激光，准备状态是将子弹拨到微型开关处，射击状
  *             态是将子弹射出，判断微型开关值，进入完成状态，完成状态通过判断一定时间
  *             微型开关无子弹认为已经将子弹射出。
  * @note       
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

#ifndef SHOOT_H
#define SHOOT_H
#include "main.h"

#include "push_motor.h"
#include "CAN_Receive.h"
#include "remote_control.h"
#include "user_lib.h"
#include "Gimbal_Task.h"
#include "trigger_switch.h"


#define SHOOT_MOTOR_TURN 1	//0代表正装  1代表反装
#define PUSH_MOTOR_TURN 0
//射击发射开关通道数据
#define Shoot_RC_Channel    1
//云台模式使用的开关通道
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  1 //1ms控制周期

#define SHOOT_FRIC_PWM_ADD_VALUE    200.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 10
//鼠标长按判断
#define PRESS_LONG_TIME 400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
//电机反馈码盘值范围
#define Half_ecd_range 4096
#define ecd_range 8191
//2006电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f  // 对于2006 传动比为36 那么该系数 =（10度/8191） *（PI/180）
#define FULL_COUNT 18
//拨叉速度
#define SHOOT_SPEED 5.0f
#define Ready_Trigger_Speed 6.0f

//6020电机RPM 变化成 旋转速度的比例
#define PUSH_Motor_RMP_TO_SPEED -0.1047f // 传动比为1 因此1*2*PI/60 
//拨轮速度
#define PUSH_SPEED -10.0f //单位rad/s


#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0
#define SWITCH_TRIGGER_OFF 1

//卡单时间 以及反转时间
#define BLOCK_TIME 700
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_TWO 1.5707963267948966192313216916398f
#define PI_Four 0.78539816339744830961566084581988f
#define PI_Ten 0.314f

//拨弹轮电机PID
#define SHOOT_SPEED_PID_KP 800.0f
#define SHOOT_SPEED_PID_KI 0.5f
#define SHOOT_SPEED_PID_KD 0.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define SHOOT_SPEED_PID_MAX_OUT 5000.0f
#define SHOOT_SPEED_PID_MAX_IOUT 2500.0f

#define PUSH_SPEED_PID_KP 1000.0f
#define PUSH_SPEED_PID_KI 0.0f
#define PUSH_SPEED_PID_KD 0.0f

#define PUSH_SPEED_PID_MAX_OUT 5000.0f
#define PUSH_SPEED_PID_MAX_IOUT 2500.0f

/* TODO1 完善结构体*/
typedef enum
{
		SHOOT_STOP = 0,
		SHOOT_WAITING,
    AMMO_OUT,
		SHOOT_INIT,
    SHOOT_READY,
		SHOOTING,
		SHOOT_JAMMED,
    SHOOT_DONE,
} shoot_system_motor_mode_e;

typedef struct
{
	const motor_measure_t *motor_measure;
	shoot_system_motor_mode_e mode;
	shoot_system_motor_mode_e last_mode;
	bool_t move_flag;
	fp32 shoot_count_flag;
	fp32 speed;
  fp32 speed_set;
  fp32 angle;
  fp32 set_angle;
  int16_t given_current;
  int8_t ecd_count;
	
} Shoot_System_Shoot_Motor_t;

typedef struct
{
	shoot_system_motor_mode_e mode;
	shoot_system_motor_mode_e last_mode;
	bool_t move_flag;
  //fp32 angle;
  //fp32 set_angle;
	
} Shoot_System_Push_Wheel_t;

typedef struct
{
	bool_t mode;	// 0或1 是否能作用于发射
		
	bool_t press_l;
  bool_t press_r;
  bool_t last_press_l;
  bool_t last_press_r;
  uint16_t press_l_time;
  uint16_t press_r_time;
  uint16_t rc_s_time;
	
}	Mouse_t;

typedef struct
{
		const RC_ctrl_t *shoot_rc_ctrl;	//遥控器指针
	
    ramp_function_source_t fric1_ramp;//摩擦轮斜波
    ramp_function_source_t fric2_ramp;
	
		Shoot_System_Shoot_Motor_t shoot_motor;			//拨叉
		Shoot_System_Push_Wheel_t push_wheel_motor;	//波轮

		Mouse_t mouse;	//鼠标
		
		u32 sys_time;	//系统时间T
	
		trigger_switch_t *PushTrigger;
		trigger_switch_t *ShootTrigger;
	
		bool_t shoot_done_flag;
	
//    bool_t move_flag;
//    uint32_t cmd_time;
//    uint32_t run_time;
//    bool_t key;
//    uint16_t key_time;
//    bool_t shoot_done;
//    uint8_t shoot_done_time;
//    int16_t BulletShootCnt;
//    int16_t last_butter_count;
} Shoot_System_t;


extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
