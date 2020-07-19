#include "GM6020_pwm.h"

void GM6020_PWM_configuration(void)	//TIM5_CH1 H10    TIM5_CH2 H11
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	//初始化TIM5
	//TIM5 CH1 H10
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	//使能PORTH时钟	
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5); //GPIOH10复用为定时器5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;           //GPIOH10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOH,&GPIO_InitStructure);              //初始化GPIOH
	
/* ----------------   PWM信号 周期和占空比的计算--------------- */
// ARR ：自动重装载寄存器的值
// CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
// PWM 信号的周期 T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// 占空比P=CCR/(ARR+1)
// 若Period = 19,则+1得20，计一次数的周期为1/CLK_CNT = 71+1/72000000 = 0.000001s,则PWM周期为20x0.000001s=0.00002s=20us=50000HZ=50KHZ
//180000 19	  
//20000/180000000	
	TIM_TimeBaseStructure.TIM_Prescaler=90-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=20000 - 1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//初始化定时器5
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1 通道1
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPE使能 
	TIM_Cmd(TIM5, ENABLE);  //使能TIM12
	
	//TIM5 CH2 H11
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5); //GPIOH11复用为定时器5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           //GPIOH11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOH,&GPIO_InitStructure);              //初始化GPIOH
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC2
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM12在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPE使能 
	TIM_Cmd(TIM5, ENABLE);  //使能TIM5
	
	TIM_SetCompare1(TIM5,1520);

}
