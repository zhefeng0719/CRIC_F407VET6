#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define STEP 2966//�����г�
#define COLD 1000//�����ȴʱ��
#define RUNX 7500//����X�����ٶ�
#define RUNY 8000//����Y���ٶ�
#define TIMERR 1000//�����ʱ������ж���ֵ/ms
#define PERR 30//������״ﶨλ�����ֵ/mm
#define VLERR 2//����ļ��ⶨλ�����ֵ/cm
#define IDPERR 8//�����ID��λ�����ֵ
#define SLOW 600//����->���㻺����ֵ/mm
#define WINDOW_SIZE 50//������ֵ�˲����ڴ�С
#define WINDOW_SIZE2 50//������ֵ�˲����ڴ�С
#define WALL 650//����ǽ�߾���
#define CARL 150//�����
enum location
{
H1 =1150,
H2 =1600,//�Ӿ���ȡʱ�ľ���	
H3 =250,//�Ӿ���ȡʱ�ľ���VL����
//����&Զ�� ̨��
H4 =350,
H5 =200,
//ץ��4����	
F1 =60,	
F2 =80,	
F3 =120,	
F4 =140,
//Ͷ��4����
G1 =120,	
G2 =140,	
G3 =160,	
G4 =180,
	
H10=500
};



#define BALANCE_TASK_PRIO		4     //Task priority //�������ȼ�
#define BALANCE_STK_SIZE 		512   //Task stack size //�����ջ��С

//Parameter of kinematics analysis of omnidirectional trolley
//ȫ����С���˶�ѧ��������
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

extern float Yaw;//ƫ����
extern int DIS[7];  // �״�����
extern int opencv[8]; // OpenCV ���ݣ�ID�����꣩
extern int openmv; // OpenMV ���ݣ�X ���꣩
extern int distance;
#endif  

