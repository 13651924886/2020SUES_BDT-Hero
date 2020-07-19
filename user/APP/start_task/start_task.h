/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       start_task.c/h
  * @brief      启动任务，将一个个任务开启，分配资源，给定任务优先级,
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
#ifndef START_TASK_H
#define START_TASK_H
#include "main.h"

#include "Detect_Task.h"
#include "Calibrate_Task.h"
#include "User_Task.h"
#include "INS_Task.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "delay.h"
#include "User_Task.h"
#include "buzzer.h"
#include "referee_usart_task.h"
#include "ANO_DT.h"

typedef struct
{
void(*task_func)(void);
uint16_t rate_hz;
uint16_t interval_ticks;
uint32_t last_run;
}sched_task_t;

//void startTast(void);
void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif
