#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define STEP 2966//步进行程
#define COLD 1000//舵机冷却时间
#define RUNX 7500//开环X轴移速度
#define RUNY 8000//开环Y轴速度
#define TIMERR 1000//允许的时间积累判断阈值/ms
#define PERR 30//允许的雷达定位误差阈值/mm
#define VLERR 2//允许的激光定位误差阈值/cm
#define IDPERR 8//允许的ID定位误差阈值
#define SLOW 600//开环->定点缓冲阈值/mm
#define WINDOW_SIZE 50//滑动均值滤波窗口大小
#define WINDOW_SIZE2 50//滑动均值滤波窗口大小
#define WALL 650//距离墙边距离
#define CARL 150//车轴距
enum location
{
H1 =1150,
H2 =1600,//视觉获取时的距离	
H3 =250,//视觉获取时的距离VL激光
//靠近&远离 台子
H4 =350,
H5 =200,
//抓料4个点	
F1 =60,	
F2 =80,	
F3 =120,	
F4 =140,
//投料4个点
G1 =120,	
G2 =140,	
G3 =160,	
G4 =180,
	
H10=500
};



#define BALANCE_TASK_PRIO		4     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小

//Parameter of kinematics analysis of omnidirectional trolley
//全向轮小车运动学分析参数
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)
void ARM1(int mode);
void ARM2(int mode);
void Step_Motor(int enable);
extern short test_num;
extern int robot_mode_check_flag;
extern u8 command_lost_count;
void Balance_task(void *pvParameters);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Get_RC(void);
void Remote_Control(void);
void Drive_Motor(float Vx,float Vy,float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);
void PS2_control(void);
float float_abs(float insert);
void robot_mode_check(void);
typedef struct
{
float get;
float target;
float kp;
float kd;
float ki;
float err;
float err_last;
float err_plus;
float err_limit;
float plus_limit;
float out;
int en;
}pid;

extern float Yaw;//偏航角
extern int DIS[7];  // 雷达数据
extern int opencv[8]; // OpenCV 数据（ID和坐标）
extern int openmv; // OpenMV 数据（X 坐标）
extern int distance;
#endif  

