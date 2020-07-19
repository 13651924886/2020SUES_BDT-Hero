/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
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
#include "main.h"

#include "stm32f4xx.h" //test22

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"//test
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"
#include "usart6.h"
#include "uart8.h"
#include "GM6020_pwm.h"

#include "exit_init.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "calibrate_task.h"
#include "remote_control.h"
#include "start_task.h"
#include "referee_usart_task.h"
#include "user_task.h"
#include "LX_IMU.h"
#include "Ano_DT.h"

#define configTICK_RATE_HZ 1000

void BSP_init(void);

int main(void)
{
		BSP_init(); //Board Support Package  硬件初始化（GPIO\DMA\)
    delay_ms(100);// 保证前面运行
		Scheduler_Setup();// 软件（变量）初始化：创建任务（线程）、创建调度器
    while (1)
    {
       Scheduler_Run();			//运行任务调度器，所有系统功能，除了中断服务函数，都在任务调度器内完成 ;
    }
}

//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //中断组 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //初始化滴答时钟 1ms SysTick中断  中断服务函数位于delay.c
    delay_init(configTICK_RATE_HZ);
    //流水灯，红绿灯初始化
    led_configuration();
    //24输出控制口 初始化
    power_ctrl_configuration();
    //摩擦轮电机PWM初始化 TIM8 CH3 CH4
    fric_PWM_configuration();
    //激光IO初始化
		laser_configuration();
		//蜂鸣器初始化 TIM12
		buzzer_init(350, 90);
    //CAN接口初始化
		CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
		//云台初始化
		GIMBAL_Setup();
		//底盘初始化
		chassis_Setup();
		//射击任务初始化
    shoot_init();
		trigger_switch_init();

		GM6020_PWM_configuration();//TIM5_CH1 H10 D   TIM5_CH2 H11 C 拨轮控制
    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
   
    remote_control_init(); 										//USART1 + DMA 遥控器初始化		
		BSPInit_CompleteBeep();
		buzzer_init(30000, 90);
		Referee_Task_Init();											//USART6 + DMA  裁判系统读取
		LX_IMU_UART8_Configuration(500000);				//UART8 陀螺仪初始化
		ANO_DT_UART7_Configuration(500000);
		ANODT_DATA_Setup();
		
		LX_ANO_DT_Init();
		GPIOF_Exti0_GPIO_Init(); //限位开关外部中断 拨叉二级弹仓内限位开关 F0 I2
		GPIOF_Exti1_GPIO_Init(); //限位开关外部中断 拨轮弹仓内限位开关 F1	I1
		//定时器6 初始化
    //TIM6_Init(60000, 90);
		//CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    //flash读取函数，把校准值放回对应参数
    //cali_param_init();
		//Calibration_Init();
}
