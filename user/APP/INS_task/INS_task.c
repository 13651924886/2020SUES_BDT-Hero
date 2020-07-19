/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       INSTask.c/h
  * @brief      主要利用陀螺仪mpu6500，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过mpu6500的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间，提供注释对应的宏定义，关闭DMA，
  *             DR的外部中断的方式.
  * @note       SPI 在陀螺仪初始化的时候需要低于2MHz，之后读取数据需低于20MHz
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

/************************* 头文件 ******************************/

#include "INS_Task.h"

#include "stm32f4xx.h"

#include "buzzer.h"
#include "timer.h"
#include "spi.h"
#include "exit_init.h"
#include "IST8310driver.h"
#include "mpu6500driver.h"
#include "mpu6500reg.h"
#include "mpu6500driver_middleware.h"

#include "AHRS.h"

#include "calibrate_Task.h"
#include "pid.h"



/************************* 宏定义 ******************************/

#define IMUWarnBuzzerOn() buzzer_on(95, 10000) //开机陀螺仪校准蜂鸣器

#define IMUWarnBuzzerOFF() buzzer_off() //开机陀螺仪校准蜂鸣器关闭

#define MPU6500_TEMPERATURE_PWM_INIT() TIM3_Init(MPU6500_TEMP_PWM_MAX, 1) //陀螺仪温度控制PWM初始化
#define IMUTempPWM(pwm) TIM_SetCompare2(TIM3, (pwm))                      //pwm给定
#define INS_GET_CONTROL_TEMPERATURE() get_control_temperate()             //获取控制温度的目标值

#if defined(MPU6500_USE_DATA_READY_EXIT)

#define MPU6500_DATA_READY_EXIT_INIT() GPIOB_Exti8_GPIO_Init() //初始化mpu6500的 外部中断 使用PB8 外部中断线 8

#define MPU6500_DATA_READY_EXIT_IRQHandler EXTI9_5_IRQHandler //宏定义外部中断函数，使用了line8外部中断

#define MPU6500_DATA_READY_EXIT_Line EXTI_Line8 //宏定义外部中断线
#endif

#if defined(MPU6500_USE_SPI) && defined(MPU6500_USE_SPI_DMA)

//宏定义初始化SPI的DMA，同时设置SPI为8位，4分频
#define MPU6500_SPI_DMA_Init(txbuf, rxbuf)                                 \
    {                                                                      \
        SPI5_DMA_Init((uint32_t)txbuf, (uint32_t)rxbuf, DMA_RX_NUM);       \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Rx, ENABLE);                   \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Tx, ENABLE);                   \
        SPI5SetSpeedAndDataSize(SPI_BaudRatePrescaler_8, SPI_DataSize_8b); \
    }

#define MPU6500_SPI_DMA_Enable() SPI5_DMA_Enable(DMA_RX_NUM) // 开始一次SPI的DMA传输
//宏定义SPI的DMA传输中断函数以及传输中断标志位
#define MPU6500_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define MPU6500_DMA_Stream DMA2_Stream5
#define MPU6500_DMA_FLAG DMA_FLAG_TCIF5
#elif defined(MPU6500_USE_SPI_DMA)
#error "the communication of mpu6500 is not SPI, can't use the DMA"
#endif


//DMA的SPI 发送的buf，以INT_STATUS开始连续读取 DMA_RX_NUM大小地址的值
#if defined(MPU6500_USE_SPI_DMA)
static const uint8_t mpu6500_spi_DMA_txbuf[DMA_RX_NUM] =
    {
        MPU_INT_STATUS | MPU_SPI_READ_MSB};
#endif


#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \

																			
																				

																		
/*********************** 全局变量定义 ***************************/
								
//处理陀螺仪，加速度计，磁力计数据的线性度，零漂
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310);
static void IMU_temp_Control(fp32 temp);

uint8_t mpu6500_spi_rxbuf[DMA_RX_NUM]; //保存接收的原始数据
//static mpu6500_real_data_t mpu6500_real_data; //转换成国际单位的MPU6500数据
mpu6500_real_data_t mpu6500_real_data; //转换成国际单位的MPU6500数据
																				
static fp32 Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度

//static fp32 gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};
fp32 gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};

//static fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //陀螺仪零漂
fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //陀螺仪零漂
static fp32 Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
static fp32 Accel_Offset[3] = {0.0f, 0.0f, 0.0f};            //加速度零漂
static ist8310_real_data_t ist8310_real_data;                //转换成国际单位的IST8310数据
static fp32 Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
                                      {0.0f, 1.0f, 0.0f},
                                      {0.0f, 0.0f, 1.0f}}; //磁力计校准线性度
//static fp32 Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //磁力计零漂
fp32 Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //磁力计零漂

static const float TimingTime = INS_DELTA_TICK * 0.001f;   //任务运行的时间 单位 s

fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

fp32 INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad 为了DEBUG方便，我把Static修饰符删掉了
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数

static const fp32 imuTempPID[3] = {MPU6500_TEMPERATURE_PID_KP, MPU6500_TEMPERATURE_PID_KI, MPU6500_TEMPERATURE_PID_KD};
static PidTypeDef imuTempPid;

static uint8_t first_temperate = 0;

//KZ_Filter()参数
fp32 k = 0;
fp32 sum = 0;
fp32 Value = 0.;
fp32 NewValue;
fp32 new_value = 0.000000000001;
fp32 kzkz = 0;
fp32 sdfor = 1.73e-05;//零漂系数，由flash写入，值越大零漂消除越好，4e-05时消除所有零漂，但是越大越容易导致操作延迟
//平滑度处理
fp32 filter_sum = 0;
#define FILTER_N 12
fp32 coe[FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // 加权系数表
fp32 sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // 加权系数和
fp32 filter_buf[FILTER_N + 1];

static fp32 KZ_Filter(void)
{
	kzkz = NewValue - INS_Angle[0];
  NewValue = INS_Angle[0];
  if((kzkz > sdfor) || (kzkz < ( - sdfor)))
  { new_value = Value;
    Value = NewValue - sum;
  }
  else
      sum += kzkz;
  
  filter_buf[FILTER_N] = Value;
  for(uint32_t i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
    filter_sum += filter_buf[i] * coe[i];
  }
  filter_sum /= sum_coe;
  k = (filter_sum /3.14)*180;//观测值，360度单位
  return filter_sum;
}

/************************ 函数内容 ******************************/

/**
  * @brief          IMU的值更新
  * @author         Stone
  * @param[in]      void
  * @retval         void
  */
void IMU_Update(void)
{
			//将读取到的mpu6500原始数据处理成国际单位的数据
        mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET), &mpu6500_real_data);
			//将读取到的ist8310原始数据处理成国际单位的数据
        ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET), &ist8310_real_data);
        //减去零漂以及旋转坐标系
        IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);


        //加速度计低通滤波
        static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
        static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
        static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
        static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


        //判断是否第一次进入，如果第一次则初始化四元数，之后更新四元数计算角度单位rad
        static uint8_t updata_count = 0;

        if( mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT)//#define MPU_DATA_READY_BIT 0 //陀螺仪数据准备
        {
						//判断是否为第一次进入
						//如果是，那么进入，第一次初始化温控和四元数
            if (updata_count == 0)
            {		
						/*IMU温控部分，暂时不用*/
                //MPU6500_TEMPERATURE_PWM_INIT();
                //PID_Init(&imuTempPid, PID_DELTA, imuTempPID, MPU6500_TEMPERATURE_PID_MAX_OUT, MPU6500_TEMPERATURE_PID_MAX_IOUT);
						/*IMU温控部分，暂时不用*/
							
                //根据加速度的数据INS_accel[]，磁力计的数据INS_mag[]进行四元数INS_quat[]初始化
                AHRS_init(INS_quat, INS_accel, INS_mag);
								//根据四元数INS_quat[]大小计算对应的欧拉角yaw(INS_Angle[0])，pitch(INS_Angle[1])，roll(INS_Angle[2])
                get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

                accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
                accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
                accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
                updata_count++;
            }
            else
            {
                //加速度计低通滤波
                accel_fliter_1[0] = accel_fliter_2[0];
                accel_fliter_2[0] = accel_fliter_3[0];

                accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

                accel_fliter_1[1] = accel_fliter_2[1];
                accel_fliter_2[1] = accel_fliter_3[1];

                accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

                accel_fliter_1[2] = accel_fliter_2[2];
                accel_fliter_2[2] = accel_fliter_3[2];

                accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

                //更新四元数
                AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);//姿态解算+互补滤波
                get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);
								KZ_Filter();

                //陀螺仪开机校准
                {
									
                    static uint16_t start_gyro_cali_time = 0;
                    if(start_gyro_cali_time == 0)
                    {
                        Gyro_Offset[0] = gyro_cali_offset[0];
                        Gyro_Offset[1] = gyro_cali_offset[1];
                        Gyro_Offset[2] = gyro_cali_offset[2];
                        start_gyro_cali_time++;
                    }
//                    else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
//                    {
//                        IMUWarnBuzzerOn();
//											
//                        if( first_temperate)
//                        {
//                            //当进入gyro_offset函数，如果无运动start_gyro_cali_time++，如果有运动 start_gyro_cali_time = 0
//                            gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
//                        }
//                    }
//                    else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
//                    {

//                        IMUWarnBuzzerOFF();
//                        start_gyro_cali_time++;
//                    }
                }       
            }          
        }           
        //IMU_temp_Control(mpu6500_real_data.temp);
}


/**
  * @brief          MPU6500和IST8310初始化
  * @author         Stone
  * @param[in]      void
  * @retval         void
  */
void IMU_Setup(void)
{
    //初始化mpu6500，失败进入死循环
		//SPI5 + DMA
    while (mpu6500_init() != MPU6500_NO_ERROR)
    {}

		//初始化ist8310，失败进入死循环
		//模拟IIC
    while (ist8310_init() != IST8310_NO_ERROR)
    {}
			
   //初始化mpu6500的数据准备的外部中断
    MPU6500_DATA_READY_EXIT_INIT();
	//初始化SPI的DMA传输的方法
    MPU6500_SPI_DMA_Init(mpu6500_spi_DMA_txbuf, mpu6500_spi_rxbuf);
}


/**
  * @brief          校准陀螺仪
  * @author         RM
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[in]      陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         返回空
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (first_temperate)
    {
        if( *time_count == 0)
        {
            Gyro_Offset[0] = gyro_cali_offset[0];
            Gyro_Offset[1] = gyro_cali_offset[1];
            Gyro_Offset[2] = gyro_cali_offset[2];
        }
        gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, time_count);

        cali_offset[0] = Gyro_Offset[0];
        cali_offset[1] = Gyro_Offset[1];
        cali_offset[2] = Gyro_Offset[2];
    }
}

/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @author         RM
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         返回空
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
}


/**
  * @brief          取得角度值指针
  * @author         Stone
  * @param[in]      void
  * @retval         INS_Angle[]数组首元素地址
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_Angle;
}

/**
  * @brief          取得陀螺仪值指针
  * @author         Stone
  * @param[in]      void
  * @retval         INS_gyro[]数组首元素地址
  */
const fp32 *get_MPU6500_Gyro_Data_Point(void)
{
    return INS_gyro;
}

/**
  * @brief          取得加速度计值指针
  * @author         Stone
  * @param[in]      void
  * @retval         INS_accel[]数组首元素地址
  */
const fp32 *get_MPU6500_Accel_Data_Point(void)
{
    return INS_accel;
}

/**
  * @brief          减去零漂以及旋转坐标系
  * @author         Stone
  * @param[in]      IMU9轴的值
  * @retval         void
  */
//Gyro_Scale_Factor[i][0]陀螺仪校准线性度
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = mpu6500->gyro[0] * Gyro_Scale_Factor[i][0] + mpu6500->gyro[1] * Gyro_Scale_Factor[i][1] + mpu6500->gyro[2] * Gyro_Scale_Factor[i][2] 
									+ Gyro_Offset[i];
        accel[i] = mpu6500->accel[0] * Accel_Scale_Factor[i][0] + mpu6500->accel[1] * Accel_Scale_Factor[i][1] + mpu6500->accel[2] * Accel_Scale_Factor[i][2]
									+ Accel_Offset[i];
        mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2]
									+ Mag_Offset[i];
    }
}

/**
  * @brief          IMU恒温控制，控制成比MCU的温度高10度
  * @author         Stone
  * @param[in]      IMU的实时温度
  * @retval         void
  */
static void IMU_temp_Control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0 ;
    if (first_temperate)
    {
        PID_Calc(&imuTempPid, temp, INS_GET_CONTROL_TEMPERATURE());
        if (imuTempPid.out < 0.0f)
        {
            imuTempPid.out = 0.0f;
        }
        tempPWM = (uint16_t)imuTempPid.out;
        IMUTempPWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        if (temp > INS_GET_CONTROL_TEMPERATURE())
        {
            temp_constant_time ++;
            if(temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperate = 1;
                imuTempPid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;

            }
        }

        IMUTempPWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}
////////////////////////////////////////////
#if defined(MPU6500_USE_DATA_READY_EXIT)

void MPU6500_DATA_READY_EXIT_IRQHandler(void)
{
    if (EXTI_GetITStatus(MPU6500_DATA_READY_EXIT_Line) != RESET)
    {

        EXTI_ClearITPendingBit(MPU6500_DATA_READY_EXIT_Line);

//如果开启DMA传输 唤醒任务由DMA中断完成
#if defined(MPU6500_USE_SPI_DMA)
        mpu6500_SPI_NS_L();
        MPU6500_SPI_DMA_Enable();
#endif
    }
}

#endif
////////////////////////////////////////////

#if defined(MPU6500_USE_SPI) && defined(MPU6500_USE_SPI_DMA)

void MPU6500_DMA_IRQHandler(void)
{
    if (DMA_GetFlagStatus(MPU6500_DMA_Stream, MPU6500_DMA_FLAG))
    {
        DMA_ClearFlag(MPU6500_DMA_Stream, MPU6500_DMA_FLAG);
        mpu6500_SPI_NS_H();
    }
}

#endif




