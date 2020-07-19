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

#include "Shoot.h"
#include "CAN_Receive.h"
#include "gimbal_behaviour.h"
#include "Detect_Task.h"
#include "pid.h"
#include "laser.h"
#include "fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "delay.h"




#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义

static const RC_ctrl_t *shoot_rc; //遥控器指针

//static PidTypeDef trigger_motor_pid;         //电机PID
PidTypeDef push_motor_pid; 
PidTypeDef shoot_motor_pid; 

//static Shoot_Motor_t trigger_motor;          //射击数据
//Shoot_Motor_t trigger_motor;          //射击数据

Shoot_System_t shoot_system;
//static shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机
//shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机

extern int16_t Push_Can_Set_Current,Shoot_Can_Set_Current;

//微动开关IO
#define Butten_Trig_Pin GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10)

extern void getTriggerMotorMeasure(motor_measure_t *motor);

///////////////////// static ////////////////////////////
//射击状态机设置
static void Shoot_Set_Mode(void);
static void Shoot_Mode_Change_Control_Transit(void);
//射击数据更新
static void Shoot_Feedback_Update(void);
//射击控制量
static void Shoot_Set_Control(void);
//PID计算
static void Shoot_Calcualte(void);
//射击控制，控制拨弹电机角度，完成一次发射
static void shoot_bullet_control(void);
//射击完成控制，判断微动开关一段时间无子弹来判断一次发射
static void shoot_done_control(void);
//射击准备控制，将子弹送到微动开关处，
static void shoot_ready_control(void);


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
		//拨叉速度PID
    static const fp32 Shoot_speed_pid[3] = {SHOOT_SPEED_PID_KP, SHOOT_SPEED_PID_KI, SHOOT_SPEED_PID_KD};
		//拨轮速度PID
		static const fp32 Push_speed_pid[3] = {	PUSH_SPEED_PID_KP, PUSH_SPEED_PID_KI ,PUSH_SPEED_PID_KD};
    //遥控器指针
		shoot_system.shoot_rc_ctrl = get_remote_control_point();
    shoot_rc = get_remote_control_point();
		//拨叉限位开关指针
		shoot_system.ShootTrigger = get_switch_point(ShootWheel);
		//拨轮限位开关指针
		shoot_system.PushTrigger = get_switch_point(PushWheel);
    //拨叉电机指针（2006）
		shoot_system.shoot_motor.motor_measure = get_Shoot_Motor_Measure_Point();
		//模式初始化
		shoot_system.push_wheel_motor.mode = SHOOT_STOP;
		shoot_system.shoot_motor.mode = SHOOT_STOP;
		//初始化系统时间
		shoot_system.sys_time = SysTick_GetTick();
		//初始化鼠标模式
		shoot_system.mouse.mode = 0;
		//初始化发射成功标志
		shoot_system.shoot_done_flag = 0;
    //初始化PID
    PID_Init(&shoot_motor_pid, PID_POSITION, Shoot_speed_pid, SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
		PID_Init(&push_motor_pid, PID_POSITION, Push_speed_pid, PUSH_SPEED_PID_MAX_OUT, PUSH_SPEED_PID_MAX_IOUT);
    //更新数据
    Shoot_Feedback_Update();
    ramp_init(&shoot_system.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);
    ramp_init(&shoot_system.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);

    shoot_system.shoot_motor.ecd_count = 0;
    shoot_system.shoot_motor.angle = shoot_system.shoot_motor.motor_measure->ecd * Motor_ECD_TO_ANGLE;
    shoot_system.shoot_motor.given_current = 0;
		shoot_system.shoot_motor.move_flag = 0;
		shoot_system.shoot_motor.shoot_count_flag = 0;
    shoot_system.shoot_motor.set_angle = shoot_system.shoot_motor.angle;
    shoot_system.shoot_motor.speed = 0.0f;
    shoot_system.shoot_motor.speed_set = 0.0f;
		
		shoot_system.push_wheel_motor.move_flag = 0;
//    triggshoot_system.pusher_motor.BulletShootCnt = 0;

}
/**
  * @brief          射击循环
  * @author         RM
  * @param[in]      void
  * @retval         返回can控制值
  */

int16_t shoot_control_loop(void)
{
    Shoot_Set_Mode();        //设置状态机
		Shoot_Mode_Change_Control_Transit();
    Shoot_Feedback_Update(); //更新数据
		Shoot_Set_Control();
		Shoot_Calcualte();		
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
u8 sig = 0;
static void Shoot_Set_Mode(void)
{
		//拨下，关闭系统，切换到STOP模式
    if((switch_is_down(shoot_system.shoot_rc_ctrl->rc.s[Shoot_RC_Channel]))&& shoot_system.shoot_motor.move_flag == 0) //|| (shoot_system.shoot_rc_ctrl->key.v & SHOOT_OFF_KEYBOARD)
		{
        shoot_system.push_wheel_motor.mode = SHOOT_STOP;
				shoot_system.shoot_motor.mode			 = SHOOT_STOP;
    }	
	   //拨上或中,启动摩擦轮，拨轮和拨叉不动
		if(	(switch_is_up(shoot_system.shoot_rc_ctrl->rc.s[Shoot_RC_Channel]) || switch_is_mid(shoot_system.shoot_rc_ctrl->rc.s[Shoot_RC_Channel]))
			&& shoot_system.shoot_motor.move_flag == 0)			 
		{
				shoot_system.push_wheel_motor.mode = SHOOT_WAITING;	// 模式切换到等待摩擦轮启动
				shoot_system.shoot_motor.mode 		 = SHOOT_WAITING;	// 模式切换到等待摩擦轮启动
		
				//启动系统，并切换到空弹模式,启动拨轮开始旋转，转速恒为PUSH_SPEED，拨叉静止不动
				if(	(shoot_system.fric1_ramp.out == shoot_system.fric1_ramp.max_value && shoot_system.fric2_ramp.out == shoot_system.fric2_ramp.max_value)&& shoot_system.shoot_motor.move_flag == 0	)
				{
						shoot_system.push_wheel_motor.mode =  AMMO_OUT;	// 模式切换到空载
						shoot_system.shoot_motor.mode 		 = 	AMMO_OUT;	// 模式切换到空载	
				}
						//系统启动后，若检测到一级弹舱限位开关触发，拨轮立即停止旋转,拨叉开始旋转进弹
				if (shoot_system.PushTrigger->trigger_state == 1 && shoot_system.shoot_motor.move_flag == 0 )
				{
						shoot_system.push_wheel_motor.mode = SHOOT_INIT;
						shoot_system.shoot_motor.mode 		 = SHOOT_INIT;					
				}
				//二级弹舱限位开关触发后，拨叉立即停止旋转，子弹已经进入二级弹舱就绪，使能鼠标功能，等待发射指令
				if (shoot_system.ShootTrigger->trigger_state == 1 && shoot_system.shoot_motor.last_mode != SHOOTING )
				{		
						sig = 1;
						shoot_system.shoot_motor.mode 		 = SHOOT_READY;
						shoot_system.push_wheel_motor.mode = SHOOT_READY;
						shoot_system.shoot_motor.set_angle = shoot_system.shoot_motor.angle;
						shoot_system.shoot_motor.set_angle = rad_format(shoot_system.shoot_motor.set_angle + PI);
						//shoot_system.mouse.mode = 1;
						if (	 
								 shoot_system.shoot_motor.move_flag == 0 
								&& shoot_system.mouse.press_l 				== 1 
								&& shoot_system.mouse.last_press_l 		== 1
		//						shoot_system.mouse.mode						== 1 
							 )
						{

								shoot_system.shoot_motor.mode = SHOOTING;
								shoot_system.shoot_motor.move_flag = 1;			
						}
					}
			}

}

static void Shoot_Mode_Change_Control_Transit(void)
{
		shoot_system.shoot_motor.last_mode 			= shoot_system.shoot_motor.mode;
		shoot_system.push_wheel_motor.last_mode = shoot_system.push_wheel_motor.mode;
}
static void Shoot_Feedback_Update(void)
{
//////////////2006电机 拨叉
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] 
										 + (shoot_system.shoot_motor.motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    shoot_system.shoot_motor.speed = -speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_system.shoot_motor.motor_measure->ecd - shoot_system.shoot_motor.motor_measure->last_ecd > Half_ecd_range)
    {
        shoot_system.shoot_motor.ecd_count--;
    }
    else if (shoot_system.shoot_motor.motor_measure->ecd - shoot_system.shoot_motor.motor_measure->last_ecd < -Half_ecd_range)
    {
        shoot_system.shoot_motor.ecd_count++;
    }

    if (shoot_system.shoot_motor.ecd_count == FULL_COUNT)
    {
        shoot_system.shoot_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_system.shoot_motor.ecd_count == -FULL_COUNT)
    {
        shoot_system.shoot_motor.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    shoot_system.shoot_motor.angle = -((shoot_system.shoot_motor.ecd_count * ecd_range + shoot_system.shoot_motor.motor_measure->ecd) * Motor_ECD_TO_ANGLE);

    //鼠标按键
    shoot_system.mouse.last_press_l = shoot_system.mouse.press_l;
    shoot_system.mouse.last_press_r = shoot_system.mouse.press_r;
    shoot_system.mouse.press_l = shoot_system.shoot_rc_ctrl->mouse.press_l;
    shoot_system.mouse.press_r = shoot_rc->mouse.press_r;
    //长按计时
    if (shoot_system.mouse.press_l)
    {
        if (shoot_system.mouse.press_l_time < PRESS_LONG_TIME)
        {
            shoot_system.mouse.press_l_time++;
        }
    }
    else
    {
        shoot_system.mouse.press_l_time = 0;
    }

    if (shoot_system.mouse.press_r)
    {
        if (shoot_system.mouse.press_r_time < PRESS_LONG_TIME)
        {
            shoot_system.mouse.press_r_time++;
        }
    }
    else
    {
        shoot_system.mouse.press_r_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_system.shoot_motor.mode != SHOOT_STOP && switch_is_down(shoot_system.shoot_rc_ctrl->rc.s[Shoot_RC_Channel]))
    {

        if (shoot_system.mouse.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_system.mouse.rc_s_time++;
        }
    }
    else
    {
        shoot_system.mouse.rc_s_time = 0;
    }
}
u8 sisi1 = 0;

u8 sisi2 = 0;

static void Shoot_Set_Control(void)
{
	if ((shoot_system.push_wheel_motor.mode == SHOOT_STOP && shoot_system.shoot_motor.mode == SHOOT_STOP) || 
		(shoot_system.push_wheel_motor.mode == SHOOT_READY && shoot_system.shoot_motor.mode == SHOOT_READY)) 
//			&& shoot_system.shoot_motor.move_flag == 0)
	{
			shoot_system.push_wheel_motor.move_flag = 0;	
			shoot_system.shoot_motor.speed_set = 0.0f;		//BUGLIST1
		
			if (shoot_system.push_wheel_motor.mode == SHOOT_STOP && shoot_system.shoot_motor.mode == SHOOT_STOP)
			{
					shoot_system.fric1_ramp.out = Fric_OFF;
					shoot_system.fric2_ramp.out = Fric_OFF;
			}
	}
	if (shoot_system.push_wheel_motor.mode == AMMO_OUT && shoot_system.shoot_motor.mode == AMMO_OUT)	
	{
			shoot_system.push_wheel_motor.move_flag = 1;	
			shoot_system.shoot_motor.speed_set = 0.0f;
	}
	if (shoot_system.push_wheel_motor.mode == SHOOT_INIT && shoot_system.shoot_motor.mode == SHOOT_INIT)
	{
			shoot_system.push_wheel_motor.move_flag = 0;	
			shoot_system.shoot_motor.speed_set 			= SHOOT_SPEED;
	}
	
	if(	shoot_system.shoot_motor.mode == SHOOTING || shoot_system.shoot_motor.move_flag == 1 )
	{			
				shoot_system.shoot_motor.shoot_count_flag++;
				shoot_system.shoot_motor.speed_set 			= SHOOT_SPEED;
				if( shoot_system.shoot_motor.shoot_count_flag >= 1000)
				{
					shoot_system.shoot_motor.move_flag = 0;
					shoot_system.shoot_motor.shoot_count_flag = 0;
					shoot_system.push_wheel_motor.mode = SHOOT_INIT;
					shoot_system.shoot_motor.mode 		 = SHOOT_INIT;
					shoot_system.PushTrigger->trigger_state = 0;
					shoot_system.ShootTrigger->trigger_state = 0;
					shoot_system.mouse.mode = 0; //发射完后失能，等待下一个子弹准备就绪
				}
			
//			if ( shoot_system.shoot_motor.set_angle > shoot_system.shoot_motor.angle)
//			{
//				if (shoot_system.shoot_motor.set_angle - shoot_system.shoot_motor.angle > 0.05f)
//				{
//						//没到达,因此一直设置旋转速度
//						shoot_system.shoot_motor.speed_set = SHOOT_SPEED;					
//				}	
//				else
//				{
//						shoot_system.push_wheel_motor.mode = SHOOT_INIT;
//						shoot_system.shoot_motor.mode 		 = SHOOT_INIT;
//						shoot_system.shoot_motor.move_flag 			 = 0;
//						shoot_system.PushTrigger->trigger_state = 0;
//						shoot_system.ShootTrigger->trigger_state = 0;
//						//shoot_system.mouse.mode = 0; //发射完后失能，等待下一个子弹准备就绪
//				}							
//			}
//			else if(shoot_system.shoot_motor.set_angle < shoot_system.shoot_motor.angle)
//			{
//				if (shoot_system.shoot_motor.angle - shoot_system.shoot_motor.set_angle > 0.05f)
//				{
//						//没到达,因此一直设置旋转速度
//						sisi2 =1;
//						shoot_system.shoot_motor.speed_set = SHOOT_SPEED;					
//				}		
//				else
//				{
//						sisi2 = 1;
//						shoot_system.push_wheel_motor.mode = SHOOT_INIT;
//						shoot_system.shoot_motor.mode 		 = SHOOT_INIT;
//						shoot_system.shoot_motor.move_flag 			 = 0;
//						shoot_system.PushTrigger->trigger_state = 0;
//						shoot_system.ShootTrigger->trigger_state = 0;
//						//shoot_system.mouse.mode = 0; //发射完后失能，等待下一个子弹准备就绪
//				}							
//			}
//					
	}
			
}

static void Shoot_Calcualte(void)
{		
	  //摩擦轮pwm
    static uint16_t fric_pwm1 = Fric_OFF;
    static uint16_t fric_pwm2 = Fric_OFF;
	
		if(shoot_system.push_wheel_motor.mode == SHOOT_STOP && shoot_system.shoot_motor.mode == SHOOT_STOP)
		{

        shoot_fric_off();
				Push_Wheel_Stop();
				Shoot_Can_Set_Current = 0;	
		}
		
    else
    {
				//shoot_laser_on();       //激光开启
				
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_system.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
				//第一个摩擦轮开启后，再开启第二个摩擦轮
        if(shoot_system.fric1_ramp.out == shoot_system.fric1_ramp.max_value)
        {
            ramp_calc(&shoot_system.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        }
				//摩擦轮还未完全启动或未启动时，拨叉和拨轮停止转动
        if( shoot_system.fric2_ramp.out != shoot_system.fric2_ramp.max_value)
        {
            shoot_system.shoot_motor.speed_set = 0.0f;
						shoot_system.push_wheel_motor.move_flag = 0;
        }
				fric_pwm1 = (uint16_t)(shoot_system.fric1_ramp.out);
        fric_pwm2 = (uint16_t)(shoot_system.fric2_ramp.out);

        shoot_fric1_on(fric_pwm1);
        shoot_fric2_on(fric_pwm2);
//	//鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
//				static uint16_t up_time = 0;
//				if (shoot_system.mouse.press_r)
//				{
//						up_time = UP_ADD_TIME;
//				}

//				if (up_time > 0)
//				{
//						shoot_system.fric1_ramp.max_value = Fric_UP;
//						shoot_system.fric2_ramp.max_value = Fric_UP;
//						up_time--;
//				}
//				else
//				{
//						shoot_system.fric1_ramp.max_value = Fric_DOWN;
//						shoot_system.fric2_ramp.max_value = Fric_DOWN;
//				}
        //计算拨弹轮电机PID
        PID_Calc(&shoot_motor_pid, shoot_system.shoot_motor.speed, shoot_system.shoot_motor.speed_set);
				
        shoot_system.shoot_motor.given_current = (int16_t)(shoot_motor_pid.out);
		#if SHOOT_MOTOR_TURN				
						Shoot_Can_Set_Current = -shoot_system.shoot_motor.given_current;			
		#else
						Shoot_Can_Set_Current = shoot_system.shoot_motor.given_current;
						
		#endif
		if(shoot_system.push_wheel_motor.move_flag == 1)
		{
			#if PUSH_MOTOR_TURN
							Push_Wheel_CCW();
			#else
							Push_Wheel_CW();
			#endif
		}
		else 
		{
			Push_Wheel_Stop();
		}

    }
}	
				
/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
//static void shoot_bullet_control(void)
//{
//    //子弹射出判断
////    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
////    {
////        trigger_motor.shoot_done = 1;
////        trigger_motor.shoot_done_time = 0;

////        shoot_mode = SHOOT_DONE;
////        trigger_motor.set_angle = trigger_motor.angle;
////    }

//    //每次拨动 1/4PI的角度
//    if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_BULLET)
//    {
//        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
//				trigger_motor.cmd_time = SysTick_GetTick();
//        trigger_motor.move_flag = 1;
//    }

//    //到达角度判断
//    if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//    {
//        //没到达一直设置旋转速度
//        trigger_motor.speed_set = TRIGGER_SPEED;
//        trigger_motor.run_time = SysTick_GetTick();

//        //堵转判断
//        if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME &&
//					trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME &&
//					fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
//        {
//            trigger_motor.speed_set = -TRIGGER_SPEED;
//        }
//        else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
//        {
//            trigger_motor.cmd_time = SysTick_GetTick();
//        }
//    }
//    else
//    {
//        trigger_motor.move_flag = 0;
//    }
//}
///**
//  * @brief          射击完成控制，判断微动开关一段时间无子弹来判断一次发射
//  * @author         RM
//  * @param[in]      void
//  * @retval         void
//  */
//static void shoot_done_control(void)
//{
//    trigger_motor.speed_set = 0.0f;
//    //射击完成判断，判断微动开关一段时间无子弹
//    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
//    {
//        if (trigger_motor.shoot_done_time < SHOOT_DONE_KEY_OFF_TIME)
//        {
//            trigger_motor.shoot_done_time++;
//        }
//        else if (trigger_motor.shoot_done_time == SHOOT_DONE_KEY_OFF_TIME)
//        {
//            trigger_motor.BulletShootCnt++;
//            shoot_mode = SHOOT_READY;
//        }
//    }
//    else
//    {
//        shoot_mode = SHOOT_BULLET;
//    }
//}
///**
//  * @brief          射击准备控制，将子弹送到微动开关处，
//  * @author         RM
//  * @param[in]      void
//  * @retval         void
//  */
//static void shoot_ready_control(void)
//{

//    if (trigger_motor.shoot_done)
//    {
//        trigger_motor.shoot_done = 0;
//    }

//    if (trigger_motor.key == SWITCH_TRIGGER_ON)
//    {
//        //判断子弹到达微动开关处
//        trigger_motor.set_angle = trigger_motor.angle;
//        trigger_motor_pid.out = 0.0f;
//        trigger_motor_pid.Iout = 0.0f;

//        trigger_motor.speed_set = 0.0f;
//        trigger_motor.move_flag = 0;
//        trigger_motor.key_time = 0;
//    }
//    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time < KEY_OFF_JUGUE_TIME)
//    {
//        //判断无子弹一段时间
//        trigger_motor.key_time++;
//    }
//    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time == KEY_OFF_JUGUE_TIME)
//    {
//        //微动开关一段时间没有子弹，进入拨弹，一次旋转 1/10PI的角度
//        if (trigger_motor.move_flag == 0)
//        {
//            trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Ten);
//            trigger_motor.cmd_time = SysTick_GetTick();
//            trigger_motor.move_flag = 1;
//        }

//        if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//        {
//            //角度达到判断
//            trigger_motor.speed_set = Ready_Trigger_Speed;
//            trigger_motor.run_time = SysTick_GetTick();
//            //堵转判断
//            if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
//            {
//                trigger_motor.speed_set = -Ready_Trigger_Speed;
//            }
//            else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
//            {
//                trigger_motor.cmd_time = SysTick_GetTick();
//            }
//        }
//        else
//        {
//            trigger_motor.move_flag = 0;
//        }
//    }
//}
