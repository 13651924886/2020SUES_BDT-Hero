#ifndef __LX_IMU_H
#define __LX_IMU_H


//==引用
#include "main.h"
#include "stm32f4xx.h"


//==定义/声明
#define FUN_NUM_LEN 256
typedef struct
{
//	void(*fun_frame_send)(void);
	u8 D_Addr; //目标地址
	u8 WTS;		 //wait to send等待发送标记
	//
//	u16 fre_hz;
	u16 fre_ms; //发送周期
	u16 time_cnt_ms; //计时变量
	
}_dt_frame_st;
//cmd
typedef struct
{
	u8 CID;
	u8 CMD[10];
}_cmd_st;

//check
typedef struct
{
	u8 ID;
	u8 SC;
	u8 AC;
}_ck_st;

//param
typedef struct
{
	u16 par_id;
	s32 par_val;
}_par_st;

//data status
typedef struct
{
	_dt_frame_st fun[FUN_NUM_LEN];
	//
	u8 wait_ck;
	//
	_cmd_st cmd_send;
	_ck_st ck_send;
	_ck_st ck_back;
	_par_st par_data;
}_dt_st;
enum 
{
	ch_1_rol=0,
	ch_2_pit,
	ch_3_thr,
	ch_4_yaw,
	ch_5_aux1,
	ch_6_aux2,
	ch_7_aux3,
	ch_8_aux4,
	ch_9_aux5,	
	ch_10_aux6,	
};

//0x40
typedef struct
{
	s16 ch_[10]; //	

}__attribute__ ((__packed__)) _rc_ch_st;

typedef union 
{
	u8 byte_data[20];
	_rc_ch_st st_data;
}_rc_ch_un;

//0x41
typedef struct
{
	s16 rol;
	s16 pit;
	s16 thr;
	s16 yaw_dps;
	s16 vel_x;
	s16 vel_y;
	s16 vel_z;

}__attribute__ ((__packed__)) _rt_tar_st;

typedef union 
{
	u8 byte_data[14];
	_rt_tar_st st_data;
}_rt_tar_un;

//0x0D
typedef struct
{
	u16 voltage_100;
	u16 current_100;

}__attribute__ ((__packed__)) _fc_bat_st;

typedef union 
{
	u8 byte_data[4];
	_fc_bat_st st_data;
}_fc_bat_un;


//
typedef struct
{
	u16 pwm_m1;
	u16 pwm_m2;
	u16 pwm_m3;
	u16 pwm_m4;
	u16 pwm_m5;
	u16 pwm_m6;
	u16 pwm_m7;
	u16 pwm_m8;
}_pwm_st;
//==数据声明
extern _dt_st dt;
extern _rt_tar_un rt_tar;
extern _fc_bat_un fc_bat;
extern _pwm_st pwm_to_esc;

//==函数声明
//static
static void ANO_DT_LX_Send_Data(u8 *dataToSend , u8 length);
static void ANO_DT_LX_Data_Receive_Anl(u8 *data,u8 len);

//public
extern void ANO_LX_Task(void);

extern void LX_ANO_DT_Init(void);

extern const fp32 *get_LX_IMU_angle_point(void);
extern const fp32 *get_LX_IMU_gyro_point(void);

extern void ANO_LX_Data_Exchange_Task(float dT_s);
extern void ANO_DT_LX_Data_Receive_Prepare(u8 data);
//
extern void CMD_Send(u8 dest_addr,_cmd_st *cmd);
extern void CK_Back(u8 dest_addr,_ck_st *ck);
extern void PAR_Back(u8 dest_addr,_par_st *par);

#endif