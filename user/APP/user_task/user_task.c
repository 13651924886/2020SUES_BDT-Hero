/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      一个普通心跳程序，如果设备无错误，绿灯1Hz闪烁,然后获取姿态角
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

#include "User_Task.h"
#include "main.h"


#include "led.h"

#include "Detect_Task.h"
#include "INS_Task.h"

#define user_is_error() toe_is_error(errorListLength)

//姿态角 单位度
//fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};

fp32 angle_degree2[3] = {0.0f, 0.0f, 0.0f};

//const volatile fp32 *angle;
const volatile short *angle2;

void UserTask_Setup(void)
{
   //获取姿态角指针
   //angle = get_INS_angle_point();
	 //angle2 = get_WT_angle_point(); 		
}

void UserTask(void)
{
        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
//        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
//        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
//        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;
//				
				//angle_degree2[0] = (*(angle2 + INS_YAW_ADDRESS_OFFSET)) / 182.0f / 57.3f; 	//PITCH 抬头增大 低头减小
				//angle_degree2[1] = (*(angle2 + INS_PITCH_ADDRESS_OFFSET)) / 182.0f/ 57.3f; //
				//angle_degree2[2] = (*(angle2 + INS_ROLL_ADDRESS_OFFSET)) / 182.0f/ 57.3f; //YAW 左大右小

//        if (!user_is_error())
//        {
//            led_green_on();
//        }
//        vTaskDelay(500);
//        led_green_off();
//        vTaskDelay(500);
}
