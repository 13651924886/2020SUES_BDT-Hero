#include "Start_Task.h"

uint32_t test_dT_1000hz[3],test_rT[6];

static void Loop_1000Hz(void)	//1ms执行一次，执行时间大约为
{
	test_dT_1000hz[0] = test_dT_1000hz[1];
	test_rT[3] = test_dT_1000hz[1] = GetSysTime_us ();
	test_dT_1000hz[2] = (u32)(test_dT_1000hz[1] - test_dT_1000hz[0]) ;//test_dT_1000hz[2] 是该任务时间片周期
//////////////////////////////////////////////////////////////////////	
//		calibrate_task();
			GIMBAL_task();			//保证云台自稳，同时接收控制指令  YAW 左大右小 ；PITCH 抬头小 低头大
			ANODT_DATA_Task();	//匿名上位机数据观察
//////////////////////////////////////////////////////////////////////	
			test_rT[4]= GetSysTime_us ();
			test_rT[5] = (u32)(test_rT[4] - test_rT[3]) ;	//test_rT[5] rT意思是RunTime 即1000HZ时间片里任务的执行时间
}

static void Loop_500Hz(void)	//2ms执行一次   姿态角速度环、电机输出
{	
			chassis_task();//保证底盘接收控制指令
			referee_usart_task();
			

}

static void Loop_200Hz(void)	//5ms执行一次  RPY计算、姿态角度环
{
				referee_tx_task();

			//static s16 _tmp = 0;
			//ANODT_SendF1(_tmp, _tmp+100, _tmp+200);
			//_tmp += 50;
}
static void Loop_100Hz(void)	//10ms执行一次，执行时间大约为20us  高度速度环、高度环
{
//			test_rT[0]= GetSysTime_us ();
////////////////////////////////////////////////////////////////////////				

////////////////////////////////////////////////////////////////////////		
//			test_rT[1]= GetSysTime_us ();
//			test_rT[2] = (u32)(test_rT[1] - test_rT[0]) ;	//test_rT[2] rT意思是RunTime 即100HZ时间片里任务的执行时间
				
}

static void Loop_50Hz(void)	//20ms执行一次   位置速度环
{	
	
}

static void Loop_20Hz(void)	//50ms执行一次
{	

}

static void Loop_2Hz(void)	//500ms执行一次
{

}
//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] = 
{
	{Loop_1000Hz,1000,  0, 0},			//void(*task_func)(void); 函数指针  指针就数据类型（int 整形常量，float，指针 地址)
	{Loop_500Hz , 500,  0, 0},			//uint16_t rate_hz;
	{Loop_200Hz , 200,  0, 0},			//uint16_t interval_ticks;
	{Loop_100Hz , 100,  0, 0},			//uint32_t last_run; 
	{Loop_50Hz  ,  50,  0, 0},
	{Loop_20Hz  ,  20,  0, 0},
	{Loop_2Hz   ,   2,  0, 0},
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for(index=0;index < TASK_NUM;index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND/sched_tasks[index].rate_hz; //  TICK_PER_SECOND = 1000
		//最短周期为1，也就是1ms
		if(sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有线程，是否应该执行

	
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = SysTick_GetTick();//获取到systime_ms的值，该值在Systick_Handler中每1ms递加1
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			
			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();

		}	 
	}
	

}

