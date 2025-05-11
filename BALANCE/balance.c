#include "balance.h"
#include "usbh_hid_joy.h"
float e[7]={-500, -300, -100, 0, 100, 300, 500};  //E����
float ec[7]={-250, -150, -50, 0, 50, 150, 250};   //EC����
//�����
float kp_table[7][7]={
	/*  PB   PM   PS   ZO   PS   PM   PB */
/*PB*/{ 20,  15,  10,   5,   0,  -5, -10},
/*PM*/{ 15,  10,   5,   0,  -5, -10, -15},
/*PS*/{ 10,   5,   0,  -5, -10, -15, -20},
/*ZO*/{  5,   0,  -5, -10, -15, -20, -25},
/*NS*/{  0,  -5, -10, -15, -20, -25, -30},
/*NM*/{ -5, -10, -15, -20, -25, -30, -35},
/*NB*/{-10, -15, -20, -25, -30, -35, -40}
};
float kd_table[7][7]={
	/*  PB   PM   PS   ZO   PS   PM   PB */
/*PB*/{ 20,  15,  10,   5,   0,  -5, -10},
/*PM*/{ 15,  10,   5,   0,  -5, -10, -15},
/*PS*/{ 10,   5,   0,  -5, -10, -15, -20},
/*ZO*/{  5,   0,  -5, -10, -15, -20, -25},
/*NS*/{  0,  -5, -10, -15, -20, -25, -30},
/*NM*/{ -5, -10, -15, -20, -25, -30, -35},
/*NB*/{-10, -15, -20, -25, -30, -35, -40}
	};
float ki_table[7][7]={
	/*  PB   PM   PS   ZO   PS   PM   PB */
/*PB*/{ 20,  15,  10,   5,   0,  -5, -10},
/*PM*/{ 15,  10,   5,   0,  -5, -10, -15},
/*PS*/{ 10,   5,   0,  -5, -10, -15, -20},
/*ZO*/{  5,   0,  -5, -10, -15, -20, -25},
/*NS*/{  0,  -5, -10, -15, -20, -25, -30},
/*NM*/{ -5, -10, -15, -20, -25, -30, -35},
/*NB*/{-10, -15, -20, -25, -30, -35, -40}
	};
//int Time_count=0; //Time variable //��ʱ����
u32 Buzzer_count1 = 0;
// Robot mode is wrong to detect flag bits
//������ģʽ�Ƿ��������־λ
int robot_mode_check_flag=0;
int initsta=0;
pid angle;
pid x;
pid y;
pid id;
pid ov;
pid vl;
int test=1;//���Ա���
int T1_sta=1; //����1״̬
int T2_sta=0;//����2״̬
float oil_x=0;//X������
float oil_y=0;//Y������
float Speed_x,Speed_y,Speed_z;//�����ٶ�
float pid_dis[4];//λ��PID�����
int Step_En;//�����������ʹ�ܱ�־
int time_ms;//��ʱ��
int time_ms2;//������ʱ��
int target_l,target_m,target_r;//����Ŀ����
float target_id;//IDλ�û�Ŀ����
int getID[4];//��ȡ���ĸ�ID
int putID[4];//���õ��ĸ�ID
int get_window[WINDOW_SIZE] = {0};//�������� 
int get_window_index = 0;  // ��ǰ���ڵ�����
int get_sum = 130;  // ����������Ԫ�صĺ�
int get_window2[WINDOW_SIZE2] = {0};//�������� 
int get_window_index2 = 0;  // ��ǰ���ڵ�����
int get_sum2 = 160;  // ����������Ԫ�صĺ�
int G[4]={180,160,140,120};
int Gend[4]={0};
//int ARM[4]={750,1250,1750,2250};
int put=3;
int temp=0;
int pan[4]={0};
int temp1=0;
int A[4]= {0};
short test_num;
u8 command_lost_count=0;//
Encoder OriginalEncoder; //Encoder raw data //������ԭʼ����     
//========== PWM���ʹ�ñ��� ==========//
u8 start_check_flag = 0;//����Ƿ���Ҫ���PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //��ǿ�ʼ���PWM
u8 clear_done_once = 0; //�����ɱ�־λ
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/
void init()//��ʼ��
{
if(initsta)return;
//����Ԥ��
	angle.kp = -420;
    angle.kd = -500;
    angle.ki = -7;
	angle.plus_limit=500;
	
	x.kp = -40;
    x.kd = -10;
    x.ki = -0.8;
	x.plus_limit=6000;
	x.err_limit=150;
	
	y.kp = -60;
    y.kd = -10;
    y.ki = -0.9;
	y.plus_limit=6000;
	y.err_limit=150;
	
	id.kp = 90;
    id.kd = 10;
    id.ki = 1.2;
	id.target = 130;	
	id.plus_limit=3000;
	id.err_limit=80;
	target_id=5;
	
	ov.kp = 50;
    ov.kd = 10;
    ov.ki = 1.1;
	ov.target = 160;	
	ov.plus_limit=2000;
	ov.err_limit=50;
	
	vl.kp = -500;
    vl.kd = -500;
    vl.ki = -7;
	vl.plus_limit=600;
	vl.err_limit=20;
//ʹ��	
	
	angle.en=1;
	id.en=0;
	ov.en=0;
//��ʼ���˲���
	for(int i=0;i<WINDOW_SIZE;i++)
		get_window[i]=id.target;
	get_sum=WINDOW_SIZE*id.target;	
	
	for(int i=0;i<WINDOW_SIZE2;i++)
		get_window2[i]=ov.target;
	get_sum2=WINDOW_SIZE2*ov.target;
//ģ��PID**********************�����ྫ�Ⱥܸߣ����Գ�����ģ������ 

//***************************************************************
//�����־λ
	initsta++;
}
void Fuzzy_Update()
{
	//����
	int index_x=0;
	int index_y=0;
	//����
	for(int i=0; i<7; i++)
		if(vl.err>e[i])index_x++;
	for(int i=0; i<7; i++)
		if((vl.err-vl.err_last)>ec[i])index_y++;
	index_x=index_x>7?7:index_x<1?1:index_x;
	index_y=index_y>7?7:index_y<1?1:index_y;
	vl.kp=kp_table[index_x][index_y];
	vl.kd=kd_table[index_x][index_y];
	vl.ki=ki_table[index_x][index_y];
}
void ARM1(int mode)//��צ
{
	if(mode)Servo_Count[2] = ARM1_ON;
	else Servo_Count[2] = ARM1_OFF;
	
	TIM8->CCR4=Servo_Count[2];//���1���ƺ���--��צ
}
void ARM2(int mode)//����
{
	if(mode)Servo_Count[3] = ARM2_ON;
	else Servo_Count[3] = ARM2_OFF;
		
	TIM8->CCR3=Servo_Count[3];//���2���ƺ���--����

}
void ARM3(int mode_arm3)//ת��
{
	static int init=0;
	if(!init){
		for(int i = 0;i<4;i++)
			{
			switch(putID[i])
				{
				case 1:A[i]=750;break;
				case 2:A[i]=1250;break;
				case 3:A[i]=1750;break;
				case 4:A[i]=2250;break;
				}
			}
		init++;
		}
	switch(mode_arm3){
		case 1:Servo_Count[5] = A[0];break;
		case 2:Servo_Count[5] = A[1];break;
		case 3:Servo_Count[5] = A[2];break;
		case 4:Servo_Count[5] = A[3];break;
	}	
	TIM8->CCR1=Servo_Count[5];//���2���ƺ���--����
}
void Step_Motor(int enable)
{	
GPIO_WriteBit(GPIOA,GPIO_Pin_6,1);//�������ʹ��
if(enable!=0)Step_En=1;//ʹ������encoder.c
else Step_En=0;           	
if(enable<0)GPIO_WriteBit(GPIOA,GPIO_Pin_3,1);
else    GPIO_WriteBit(GPIOA,GPIO_Pin_3,0);
}	
void PID_YAW() // �ǶȻ�
{    
    angle.err=angle.target-Yaw;	
	if(angle.err>180)angle.err-=360;//�������
	else if(angle.err<-180)angle.err+=360;
	angle.err= angle.err>50?50:angle.err<-50?-50:angle.err;//����޷�
    if (angle.err<10&&angle.err>-10)
        angle.err_plus+=angle.err; // ���ַ���
    angle.err_plus=angle.err_plus>angle.plus_limit?angle.plus_limit:angle.err_plus<-angle.plus_limit?-angle.plus_limit:angle.err_plus;
    if(angle.en)angle.out=angle.kp*angle.err+angle.kd*(angle.err-angle.err_last)+angle.ki*angle.err_plus;
	else angle.out=0;
    angle.err_last=angle.err;
}
float PID_X(float get,float target) // λ�û�x
{	
    x.err=target-get;
	if(target==0)x.err=0;//�������	
	if(get>3500)x.err=x.err_last;
	else if(get<10)x.err=x.err_last;
	x.err=x.err>x.err_limit?x.err_limit:x.err<-x.err_limit?-x.err_limit:x.err;//����޷�
    if(x.err<50&&x.err>-50)x.err_plus+=x.err; // ���ַ���    
    x.err_plus=x.err_plus>x.plus_limit?x.plus_limit:x.err_plus<-x.plus_limit?-x.plus_limit:x.err_plus;
    x.out=x.kp*x.err+x.kd*(x.err-x.err_last)+x.ki*x.err_plus;
    x.err_last=x.err;
	return x.out;	
}
float PID_Y(float get,float target) // λ�û�y
{	
    y.err=target-get;
	if(target==0)y.err=0;//�������	
	if(get>3500)y.err=y.err_last;
	else if(get<10)y.err=y.err_last;
	y.err=y.err>y.err_limit?y.err_limit:y.err<-y.err_limit?-y.err_limit:y.err;//����޷�
    if(y.err<50&&y.err>-50)y.err_plus+=y.err; // ���ַ���    
    y.err_plus=y.err_plus>y.plus_limit?y.plus_limit:y.err_plus<-y.plus_limit?-y.plus_limit:y.err_plus;
    y.out=y.kp*y.err+y.kd*(y.err-y.err_last)+y.ki*y.err_plus;
    y.err_last=y.err;
	return y.out;
}
float PID_ID() // IDλ�û�
{
	id.get=id.target;    
	for(int i=0;i<8;i++)
		if(opencv[i]==target_id)id.get=opencv[i+1];//����opencv���飬�ҵ�Ŀ��ID
//������ֵ�˲�-------------------------------------------------------	
	get_sum-=get_window[get_window_index];  // ��ȥ��ǰ�����еľ�ֵ
    get_window[get_window_index] = id.get;  // ���µ� id.get ���봰��
    get_sum+=id.get;  // ���µ�ֵ�ӵ��ܺ�
    get_window_index=(get_window_index+1)%WINDOW_SIZE;   // ���´�������
    id.get = get_sum / WINDOW_SIZE;  // ���㴰�ڵľ�ֵ
//-------------------------------------------------------------------	
    id.err=id.target-id.get;
	if(!id.en)id.err=0;//ʹ��
	id.err=id.err>id.err_limit?id.err_limit:id.err<-id.err_limit?-id.err_limit:id.err;//����޷�
    if(id.err<30&&id.err>-30)id.err_plus+=id.err; // ���ַ���    
    id.err_plus=id.err_plus>id.plus_limit?id.plus_limit:id.err_plus<-id.plus_limit?-id.plus_limit:id.err_plus;//�����޷�
    id.out=id.kp*id.err+id.kd*(id.err-id.err_last)+id.ki*id.err_plus;
    id.err_last=id.err;	
	return id.out;
}
float PID_OV() // λ�û�ov
{	
    ov.get=ov.target;
	if(ov.target==0)ov.err=0;//�������	
//������ֵ�˲�-------------------------------------------------------
	if(openmv!=0)ov.get=openmv;
	get_sum2-=get_window2[get_window_index2];  // ��ȥ��ǰ�����еľ�ֵ
    get_window2[get_window_index2] = ov.get;  // ���µ� ov.get ���봰��
    get_sum2+=ov.get;  // ���µ�ֵ�ӵ��ܺ�
    get_window_index2=(get_window_index2+1)%WINDOW_SIZE2;   // ���´�������
    ov.get = get_sum / WINDOW_SIZE2;  // ���㴰�ڵľ�ֵ
//-------------------------------------------------------------------
	ov.err=ov.target-ov.get;
	if(!ov.en)ov.err=0;//ʹ��
	ov.err=ov.err>ov.err_limit?ov.err_limit:ov.err<-ov.err_limit?-ov.err_limit:ov.err;//����޷�
    if(ov.err<30&&ov.err>-30)ov.err_plus+=ov.err; // ���ַ���    
    ov.err_plus=ov.err_plus>ov.plus_limit?ov.plus_limit:ov.err_plus<-ov.plus_limit?-ov.plus_limit:ov.err_plus;
    ov.out=ov.kp*ov.err+ov.kd*(ov.err-ov.err_last)+ov.ki*ov.err_plus;
    ov.err_last=ov.err;
	return ov.out;
}
float PID_VL() // λ�û���������
{	
    vl.err=vl.target-distance;
	if(vl.target==0)vl.err=0;//�������	
	if(distance>400)vl.err=vl.err_last;
	else if(distance<1)vl.err=vl.err_last;
	vl.err=vl.err>vl.err_limit?vl.err_limit:vl.err<-vl.err_limit?-vl.err_limit:vl.err;//����޷�
    if(vl.err<10&&vl.err>-10)vl.err_plus+=vl.err; // ���ַ���    
    vl.err_plus=vl.err_plus>vl.plus_limit?vl.plus_limit:vl.err_plus<-vl.plus_limit?-vl.plus_limit:vl.err_plus;
    vl.out=vl.kp*vl.err+vl.kd*(vl.err-vl.err_last)+vl.ki*vl.err_plus;
    vl.err_last=vl.err;
	return vl.out;
}
int ARM_GET1(int i)//��չ�ֱۣ����̹�iλ
{
	time_ms+=10;	
	if(time_ms>=0)
	{
	ARM1(0);
	ARM2(0);	
	Step_Motor(1);
	}
	if(time_ms>=STEP)
	{
	ARM1(0);
	ARM2(1);
	ARM3(i);
	Step_Motor(0);
	}
	if(time_ms>=STEP+COLD)
	{
	time_ms=0;
	return 1;	
	}		
	return 0; 
}
int ARM_GET2()//ץȡĿ�꣬��������
{
	time_ms+=10;	
	if(time_ms>=0)
	{
	ARM1(1);
	ARM2(1);	
	Step_Motor(0);
	}
	if(time_ms>=COLD)
	{
	ARM1(1);
	ARM2(0);
	Step_Motor(0);
	}
	if(time_ms>=2*COLD)
	{
	ARM1(1);
	ARM2(0);
	Step_Motor(-1);
	}
	if(time_ms>=2*COLD+STEP)
	{
	ARM1(0);
	ARM2(0);
	Step_Motor(0);
	}
	if(time_ms>=3*COLD+STEP)
	{
	time_ms=0;
	return 1;	
	}		
	return 0; 
}
int ARM_GETFIN()//ץȡĿ�꣬��������
{
	time_ms+=10;	
	if(time_ms>=0)
	{
	ARM1(1);
	ARM2(1);	
	Step_Motor(0);
	}
	if(time_ms>=COLD)
	{
	ARM1(1);
	ARM2(0);
	Step_Motor(0);
	}
	if(time_ms>=2*COLD)
	{
	ARM1(1);
	ARM2(0);
	Step_Motor(-1);
	}
	if(time_ms>=2*COLD+STEP)
	{
	ARM1(1);
	ARM2(0);
	Step_Motor(0);
	return 1;
	}		
	return 0; 
}
int ARM_PUT1()
{
time_ms+=10;	
	if(time_ms>=0)
	{
	ARM1(1);
	ARM2(0);
	Step_Motor(0);	
	}
	if(time_ms>=COLD)
	{
	ARM1(1);
	ARM2(0);
	Step_Motor(1);	
	}
	if(time_ms>=STEP+COLD)
	{
	ARM1(1);
	ARM2(1);
	Step_Motor(0);					
	}
	if(time_ms>=STEP+2*COLD)
	{
	ARM1(1);
	ARM2(1);
	Step_Motor(0);		
	return 1;			
	}		
	return 0;
}
int ARM_PUT2(int i)
{
time_ms+=10;	
	if(time_ms>=0)
	{
	ARM1(0);
	ARM2(1);
	Step_Motor(0);	
	}
	if(time_ms>=COLD)
	{
	ARM1(0);
	ARM2(0);
	Step_Motor(0);	
	}
	if(time_ms>=2*COLD)
	{
	ARM1(0);
	ARM2(0);
	ARM3(i);
	Step_Motor(-1);	
	}
	if(time_ms>=STEP+2*COLD)
	{
	ARM1(0);
	ARM2(0);
	Step_Motor(0);
	return 1;
	}			
	return 0;
}
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
	float amplitude=3.5; //Wheel target speed limit //����Ŀ���ٶ��޷�

	Vx=target_limit_float(Vx,-amplitude,amplitude);
	Vy=target_limit_float(Vy,-amplitude,amplitude);
	Vz=target_limit_float(Vz,-amplitude,amplitude);
	
	//Speed smoothing is enabled when moving the omnidirectional trolley
	//ȫ���ƶ�С���ſ����ٶ�ƽ������
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
	{
		if(Allow_Recharge==0)
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //�������ٶȽ���ƽ������
		else
			smooth_control.VX=Vx,     
			smooth_control.VY=Vy,
			smooth_control.VZ=Vz;

		//Get the smoothed data 
		//��ȡƽ�������������			
		Vx=smooth_control.VX;     
		Vy=smooth_control.VY;
		Vz=smooth_control.VZ;
	}
		
	//Mecanum wheel car
	//�����ķ��С��
	if (Car_Mode==Mec_Car||Car_Mode==Mec_Car_V550) 
	{
		//Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);

		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
	} 
		
	//Omni car
	//ȫ����С��
	else if (Car_Mode==Omni_Car) 
	{
		//Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
		MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;

		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=0;	//Out of use //û��ʹ�õ�
	}
		
	//Ackermann structure car
	//������С��
	else if (Car_Mode==Akm_Car) 
	{
		//Ackerman car specific related variables //������С��ר����ر���
		float R, Ratio=636.56, AngleR, Angle_Servo;
		
		// For Ackerman small car, Vz represents the front wheel steering Angle
		//���ڰ�����С��Vz������ǰ��ת��Ƕ�
		AngleR=Vz;
		R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
		
		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		//ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
		AngleR=target_limit_float(AngleR,-0.49f,0.32f);
		
		//Inverse kinematics //�˶�ѧ���
		if(AngleR!=0)
		{
			MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
			MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
		}
		else 
		{
			MOTOR_A.Target = Vx;
			MOTOR_B.Target = Vx;
		}
		// The PWM value of the servo controls the steering Angle of the front wheel
		//���PWMֵ���������ǰ��ת��Ƕ�
		Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
		Servo=SERVO_INIT + (Angle_Servo - 1.572f)*Ratio;

		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
		MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
		Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //���PWMֵ�޷�
	}
		
	//Differential car
	//����С��
	else if (Car_Mode==Diff_Car) 
	{
		//Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
		MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�

		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
		MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
	}
		
	//FourWheel car
	//������
	else if(Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550) 
	{	
		//Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
		MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
		MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
		MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
				
		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
	}

	//Tank Car
	//�Ĵ���
	else if (Car_Mode==Tank_Car) 
	{
		//Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
		MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�

		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
		MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
	}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	u32 lastWakeTime = getSysTickCnt();

    while(1)
    {	
		// This task runs at a frequency of 100Hz (10ms control once)
		//��������100Hz��Ƶ�����У�10ms����һ�Σ�
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 

		//Time count is no longer needed after 30 seconds
		//ʱ�������30�������Ҫ
		if(SysVal.Time_count<3000) SysVal.Time_count++;
			Buzzer_count1++;
		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
		Get_Velocity_Form_Encoder();   	
			//Click the user button to update the gyroscope zero
			//�����û������������������
			Key(); 			
		if( Allow_Recharge==1 )
			if( Get_Charging_HardWare==0 ) Allow_Recharge=0,Find_Charging_HardWare();
		
		
		
//---------------------------------------------------�û���-----------------------------------------------------------------------    		
		if(Check==0) //If self-check mode is not enabled //���û�������Լ�ģʽ
		{
			Drive_Motor(Move_X, Move_Y, Move_Z);//CAN������1������3(ROS)������5����ֱ�ӵõ�����Ŀ���ٶȣ�������⴦��			
			if(APP_ON_Flag)Get_RC();         //Handle the APP remote commands //���ȴ���APPң������

			//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
			//and the software failure flag is 0
			//�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ����������ʧ�ܱ�־λΪ0
			if(Turn_Off(Voltage)==0||(Allow_Recharge&&EN&&!Flag_Stop)) 
			{
				//��ʼ��
				init();	
				//ģ��PID����������
//				Fuzzy_Update();
				//PID����
				PID_YAW();//�ǶȻ�����
//				PID_ID();//ID��
//				PID_OV();//OPENMV��	
				
				pid_dis[0] = PID_Y(DIS[0],target_l);//��
				pid_dis[1] = PID_X(DIS[3],target_m);//��
				pid_dis[2] = PID_Y(DIS[6],target_r);//��	
				pid_dis[3] = PID_VL();//��**������
				//û�ùٷ����ٶȻ����Լ�д�˸��ںϵĶ໷
				//APP��PS2������ROS�õ���Ŀ���ٶȣ�ͳһת�����д���
				Speed_x=10000*Move_X+pid_dis[1]-pid_dis[3]				 	+oil_x;
				Speed_y=12000*Move_Y-pid_dis[2]+pid_dis[0]  			    +oil_y;
				Speed_z=10000*Move_Z;
				if(Move_Z>0)angle.target++;
				else if (Move_Z<0)angle.target--;
				//�Ƕȹ�һ������
				if(angle.target>=180)angle.target-=360;
				if(angle.target<=-180)angle.target+=360;				
				//Speed closed-loop control to calculate the PWM value of each motor, 
				//PWM represents the actual wheel speed					 
				//�ٶȱջ����Ƽ�������PWMֵ��PWM��������ʵ��ת��
				MOTOR_A.Motor_Pwm=-angle.out-Speed_x-Speed_y;//Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
				MOTOR_B.Motor_Pwm=-angle.out-Speed_x+Speed_y;//Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
				MOTOR_C.Motor_Pwm=+angle.out-Speed_x-Speed_y;//Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
				MOTOR_D.Motor_Pwm=+angle.out-Speed_x+Speed_y;//Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
				
				Limit_Pwm(16700);

				//����Ƿ���Ҫ���PWM���Զ�ִ������
				auto_pwm_clear();
				
				//Set different PWM control polarity according to different car models
				//���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ���
				switch(Car_Mode)
				{
					case Mec_Car:case Mec_Car_V550:
						Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //�����ķ��С��
					case Omni_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //ȫ����С��
					case Akm_Car:       Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //������С��
					case Diff_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //���ֲ���С��
					case FourWheel_Car:case FourWheel_Car_V550:
						Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //������ 
					case Tank_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //�Ĵ���
				}			
			
			if(T1_sta==1)
			{
			    target_l=0;
				target_m=0;
				target_r=0;
				oil_x=0;
				time_ms+=10;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			
			}
			if(T1_sta==2)//�ұ߾�ǽ1/2�������룬ǰ����A��
			{
				target_l=0;
				target_m=0;
				target_r=WALL;
				oil_x=RUNX;
				if(DIS[3]<WALL+SLOW)time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==3)//ȷ������A���ȶ�����
			{
			    target_l=0;
				target_m=WALL;
				target_r=0;
				oil_x=0;
				if((DIS[3]>WALL-PERR)&&(DIS[3]<WALL+PERR))time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}			
			}
			if(T1_sta==4)//����ת90��
			{
				target_l=0;
				target_m=0;
				target_r=0;
				if(angle.target<90)angle.target++;
				else time_ms+=10;
				if(time_ms>3*TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==5)//ת��������ұ߾�ǽ1/2�������룬��B��ǰ�������ӽ�ʱ�����ջ�����߾�ǽ1/2��������
			{
				target_m=0;
				if(DIS[3]<2200)time_ms2+=10;
				else time_ms2=0;
				if(time_ms2>TIMERR)
				{
					target_l=WALL;
					target_r=0;
				}
				else 
				{
					target_l=0;
					target_r=WALL;
				}
				oil_x=RUNX;
				if(DIS[3]<WALL-CARL+SLOW)time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					time_ms2=0;
					T1_sta++;
					}
				
			}
			if(T1_sta==6)//ȷ�����ȶ���B��
			{
				target_l=0;
				target_m=WALL-CARL;
				target_r=0;
				oil_x=0;
				if((DIS[3]>WALL-CARL-PERR)&&(DIS[3]<WALL-CARL+PERR))time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==7)//����ת90��
			{
				target_l=0;
				target_m=0;
				target_r=0;
				if(angle.target>0)angle.target--;
				else time_ms+=10;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==8)//��߾�ǽ1/2�������룬ǰ����C��
			{
				target_l=WALL;
				target_m=0;
				target_r=0;
				oil_x=RUNX;
				if(DIS[3]<WALL-CARL+SLOW)time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==9)//ȷ�����ȶ���C��
			{
				target_l=0;
				target_m=WALL-CARL;
				target_r=0;
				oil_x=0;
				if((DIS[3]>WALL-CARL-PERR)&&(DIS[3]<WALL-CARL+PERR))time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==10)//����ת90��
			{
				target_l=0;
				target_m=0;
				target_r=0;
				oil_x=0;
				if(angle.target>-90)angle.target--;
				else time_ms+=10;
				if(time_ms>3*TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==11)//��ǰ�ƶ���������ٴ�
			{
				target_l=WALL;
				target_m=0;
				target_r=0;
				oil_x=RUNX;
				if(DIS[3]<H1+SLOW)time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==12)//�Ծ���F��Ϊ���գ�ͣ��һ������λ�ã�ʹ����Y��ͶӰ���ᱻ��ǽ�ڵ�
			{
				target_l=0;
				target_m=H1;
				target_r=0;
				oil_x=0;
				if((DIS[3]>H1-PERR)&&(DIS[3]<H1+PERR))time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta++;
					}		
			}
			if(T1_sta==13)//����ƽ�ƣ�ֱ�������ά����ʵ�λ��
			{
				target_l=0;
				target_m=H1;
				target_r=0;
				oil_y=-RUNY;
				if(DIS[6]<H2+SLOW)time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR/2)
					{
					time_ms=0;
					T1_sta++;
					}
			}
			if(T1_sta==14)//ȷ���ȶ������λ��,�˿̽�������һ�����������
			{
				target_l=0;
				target_m=0;
				target_r=H2;
				oil_y=0;
				if((DIS[6]>H2-PERR)&&(DIS[6]<H2+PERR))time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T1_sta=0;
					T2_sta++;
					}
			}
			
			if(T2_sta==1)
			{	
				time_ms+=10;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}
			}
			if(T2_sta==2)//����ƣ�Ѱ��һ�������ռ������ж�ά��
			{
				temp++;
				target_l=0;
				target_m=0;
				target_r=H2;
				vl.target=H3;
				vl.target+=temp;
				if((opencv[0]!=0)&(opencv[2]!=0)&(opencv[4]!=0)&(opencv[6]!=0))
				{
					time_ms=0;
					time_ms2=0;
					getID[0]=opencv[0];
					getID[1]=opencv[2];
					getID[2]=opencv[4];
					getID[3]=opencv[6];
					oil_x=0;
					T2_sta++;
				}			
			}	
			if(T2_sta==3)//ת��Ѱ�ҷ�����
			{
				target_l=0;
				target_m=0;
				target_r=0;
				vl.target=0;
				angle.en=1;
				if(angle.target<0)angle.target++;
				else time_ms+=10;
				if(time_ms>3*TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}
			}			
				if(T2_sta==4)//Ѱ��һ�������ռ������ж�ά��
			{
				target_l=0;
				target_m=0;
				target_r=0;
				angle.en=0;
				vl.target=130;
				if((opencv[0]!=0)&(opencv[2]!=0)&(opencv[4]!=0)&(opencv[6]!=0))
				{
					time_ms=0;
					time_ms2=0;
					putID[0]=opencv[0];
					putID[1]=opencv[2];
					putID[2]=opencv[4];
					putID[3]=opencv[6];
					for(int i=0;i<4;i++)
					{
						for(int j=0;j<4;j++)
						{
							if(getID[i]==putID[j])
							{
								Gend[i]=G[j];
								break;
							}
						}
					}
					oil_x=0;
					T2_sta++;
				}			
			}	
				if(T2_sta==5)//ת����
			{
				target_l=0;
				target_m=0;
				target_r=0;
				vl.target=0;
				angle.en=1;
				if(angle.target>-90)angle.target--;
				else time_ms+=10;
				if(time_ms>3*TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}
			}			
			if(T2_sta==6)
			{
				target_l=0;
				target_m=0;
				target_r=0;
				angle.en=1;
				vl.target=F1;
				if((distance>F1-VLERR)&&(distance<F1+VLERR))time_ms+=10;
				
				if(time_ms>TIMERR){
				T2_sta++;
				time_ms=0;
				}
			}	
			if(T2_sta==7)//������ά���ƶ�
			{
				target_l=0;
				target_m=0;
				target_r=0;
				vl.target=F1;
				id.en=0;
				angle.en=1;
				angle.target=-90;
				oil_y=-RUNY/1.2;
				if(DIS[6]<H4+SLOW)time_ms+=10;
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}		
			}
			if(T2_sta==8)//��λ����һ��Ҫץȡ�ķ��鴦
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=F1;
				oil_y=0;
				if((DIS[6]<H4+PERR)&&(DIS[6]>H4-PERR)&&(distance>F1-VLERR)&&(distance<F1+VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==9)//��չ����׼��ץȡ
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=F1;
				if(ARM_GET1(getID[3]))T2_sta++;
			}
			if(T2_sta==10)//�������飬����λ
			{
				target_l=0;
				target_m=0;
				target_r=0;
				vl.target=F1;
				if(DIS[6]>H5)
					{
					oil_y=-RUNY/2;
					time_ms=0;
					}
				else{
					oil_y=0;
					if((distance<F1+VLERR)&&(distance>F1-VLERR))					
						time_ms+=10;
					else time_ms=0;
					}
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}		
			}
			if(T2_sta==11)//ץȡ��
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=F1;
				if(ARM_GET2())T2_sta++;
			}
			if(T2_sta==12)//�˶����ڶ���ץȡ�ķ��鴦
			{
				target_l=0;
				target_m=0;
				target_r=H4;//����
				vl.target=F2;
				if((DIS[6]<H4+PERR)&&(DIS[6]>H4-PERR)&&(distance>F2-VLERR)&&(distance<F2+VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}		
			}
			if(T2_sta==13)//��չ����׼��ץȡ
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=F2;
				if(ARM_GET1(getID[2]))T2_sta++;		
			}
			if(T2_sta==14)//�����ڶ������飬����λ
			{
				target_l=0;
				target_m=0;
				target_r=0;
				vl.target=F2;
				if(DIS[6]>H5)
					{
					oil_y=-RUNY/2;
					time_ms=0;
					}
				else{
					oil_y=0;
					if((distance<F2+VLERR)&&(distance>F2-VLERR))					
						time_ms+=10;
					else time_ms=0;
					}
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}		
			}
			if(T2_sta==15)//ץȡ��
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=F2;
				if(ARM_GET2())T2_sta++;		
			}
			if(T2_sta==16)//�˶������������鴦
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=F3;
				if((DIS[6]<H4+PERR)&&(DIS[6]>H4-PERR)&&(distance>F3-VLERR)&&(distance<F3+VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==17)//��չ����׼��ץȡ
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=F3;
				if(ARM_GET1(getID[1]))T2_sta++;			
			}
			if(T2_sta==18)//�������������飬����λ
			{
				target_l=0;
				target_m=0;
				target_r=0;
				vl.target=F3;
				if(DIS[6]>H5)
					{
					oil_y=-RUNY/2;
					time_ms=0;
					}
				else{
					oil_y=0;
					if((distance<F3+VLERR)&&(distance>F3-VLERR))
					{
					time_ms+=10;
					}
					else time_ms=0;
					}
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}			
			}
			if(T2_sta==19)//ץȡ��
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=F3;
				if(ARM_GET2())T2_sta++;			
			}
			if(T2_sta==20)//�˶������ĸ����鴦
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=F4;				
				if((DIS[6]<H4+PERR)&&(DIS[6]>H4-PERR)&&(distance<F4+VLERR)&&(distance>F4-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==21)//��չ����׼��ץȡ
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=F4;	
				if(ARM_GET1(getID[0]))T2_sta++;			
			}
			if(T2_sta==22)//�������ĸ����飬����λ
			{
				target_l=0;
				target_m=0;
				target_r=0;
				vl.target=F4;	
				if(DIS[6]>H5)
					{
					oil_y=-RUNY/2;
					time_ms=0;
					}
				else{
					oil_y=0;
					if((distance<F4+VLERR)&&(distance>F4-VLERR))
					{
					time_ms+=10;
					}
					else time_ms=0;
					}
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==23)//ץȡ��
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=F4;	
				if(ARM_GETFIN())T2_sta++;		
			}
			if(T2_sta==24)//�������ץȡ,׼��ǰ��E��
			{
				target_l=0;
				target_m=H10-CARL;
				target_r=0;
				vl.target=0;
				if((DIS[3]>H10-CARL-PERR)&&(DIS[3]<H10-CARL+PERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==25)//��ת90��
			{
				target_l=0;
				target_m=0;
				target_r=0;	
				vl.target=0;
				if(angle.target<0)angle.target++;
				else time_ms+=10;
				if(time_ms>3*TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}
			}
			if(T2_sta==26)//��ǰ��λ
			{
				target_l=0;
				target_m=0;
				target_r=0;
				vl.target=Gend[3];
				if((distance>Gend[3]-VLERR)&&(distance<Gend[3]+PERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==27)//��λ�õ�һ�����õ�
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=Gend[3];
				if((DIS[6]<H4+PERR)&&(DIS[6]>H4-PERR)&&(distance<Gend[3]+VLERR)&&(distance>Gend[3]-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==28)//�Ӻö�����չ����
			{
			target_l=0;
			target_m=0;
			target_r=H4;
			if(ARM_PUT1())T2_sta++;	
			}
			if(T2_sta==29)//�������õ�
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=Gend[3];
				if((DIS[6]<H5+PERR)&&(DIS[6]>H5-PERR)&&(distance<Gend[3]+VLERR)&&(distance>Gend[3]-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}
			}
			if(T2_sta==30)//����
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=Gend[3];
				if(ARM_PUT2(putID[2]))T2_sta++;
			}
			if(T2_sta==31)//��λ�õڶ������õ�
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=Gend[2];
				if((DIS[6]<H4+PERR)&&(DIS[6]>H4-PERR)&&(distance<Gend[2]+VLERR)&&(distance>Gend[2]-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==32)//�Ӻö�����չ����
			{
			target_l=0;
			target_m=0;
			target_r=H4;
			if(ARM_PUT1())T2_sta++;	
			}
			if(T2_sta==33)//�������õ�
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=Gend[2];
				if((DIS[6]<H5+PERR)&&(DIS[6]>H5-PERR)&&(distance<Gend[2]+VLERR)&&(distance>Gend[2]-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}
			}
			if(T2_sta==34)//����
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=Gend[2];
				if(ARM_PUT2(putID[1]))T2_sta++;
			}
			if(T2_sta==35)//��λ�õ��������õ�
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=Gend[1];
				if((DIS[6]<H4+PERR)&&(DIS[6]>H4-PERR)&&(distance<Gend[1]+VLERR)&&(distance>Gend[1]-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==36)//�Ӻö�����չ����
			{
			target_l=0;
			target_m=0;
			target_r=H4;
			if(ARM_PUT1())T2_sta++;	
			}
			if(T2_sta==37)//�������õ�
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=Gend[1];
				if((DIS[6]<H5+PERR)&&(DIS[6]>H5-PERR)&&(distance<Gend[1]+VLERR)&&(distance>Gend[1]-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}
			}
			if(T2_sta==38)//����
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=Gend[1];
				if(ARM_PUT2(putID[0]))T2_sta++;
			}
			if(T2_sta==39)//��λ�õ��ĸ����õ�
			{
				target_l=0;
				target_m=0;
				target_r=H4;
				vl.target=Gend[0];
				if((DIS[6]<H4+PERR)&&(DIS[6]>H4-PERR)&&(distance<Gend[0]+VLERR)&&(distance>Gend[0]-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}	
			}
			if(T2_sta==40)//�Ӻö�����չ����
			{
			target_l=0;
			target_m=0;
			target_r=H4;
			if(ARM_PUT1())T2_sta++;	
			}
			if(T2_sta==41)//�������õ�
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=Gend[0];
				if((DIS[6]<H5+PERR)&&(DIS[6]>H5-PERR)&&(distance<Gend[0]+VLERR)&&(distance>Gend[0]-VLERR))
				{
				time_ms+=10;
				}
				else time_ms=0;
				if(time_ms>TIMERR)
					{
					time_ms=0;
					T2_sta++;
					}
			}
			if(T2_sta==42)//����
			{
				target_l=0;
				target_m=0;
				target_r=H5;
				vl.target=Gend[0];
				if(ARM_PUT2(putID[0]))T2_sta++;
			}
			if(T2_sta==43)//
			{
				target_l=0;
				target_m=H10;
				target_r=H10;
				vl.target=0;		
			}
//			if(T2_sta==28)//
//			{
//						
//			}
			if(T2_sta==100)//
			{
						
			}
			
			if(T1_sta==99)//
			{
//				target_l=0;
//				target_m=300;
//				target_r=300;
			vl.target=120;
          
			
			}
			if(T2_sta==100)//
			{
						
			}
			if(T2_sta==101)//
			{
									
			}
			if(T2_sta==102)//
			{
									
			}
			
			
			
			
						
		
//---------------------------------------------------		
			}
			//If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
			//���Turn_Off(Voltage)����ֵΪ1������������С�������˶���PWMֵ����Ϊ0
			else	Set_Pwm(0,0,0,0,0); 
		}
//---------------------------------------------------�û���-----------------------------------------
		
		
		
		
		
		
//***********************��*********��*********ģ********ʽ**********************************************	
		else						
			{
				if(Proc_Flag==3)						//�Լ���
				{
					 if(check_time_count_motor_forward>0)
					 {	 
						 check_time_count_motor_forward--;
						 Full_rotation=16799;
					 }
					 else if(check_time_count_motor_retreat>0) 
					 {	 
							check_time_count_motor_retreat--;
						 Full_rotation=-16799;
					 }		

					 switch(Car_Mode)
					 {
						    case Mec_Car:case Mec_Car_V550: 
							    Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //Mecanum wheel car       //�����ķ��С��
							case Omni_Car:      Set_Pwm(-Full_rotation,  Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //Omni car                //ȫ����С��
							case Akm_Car:       Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Ackermann structure car //������С��
							case Diff_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Differential car        //���ֲ���С��
							case FourWheel_Car:case FourWheel_Car_V550:
								Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0    ); break; //FourWheel car           //������ 
							case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0    ); break; //Tank Car                //�Ĵ���
					 } 
					 if(!(check_time_count_motor_retreat>0) && !(check_time_count_motor_forward>0))
					 {	 
						 Set_Pwm(0,0,0,0,0);		 
					 }
				}
				if(Proc_Flag==4)		Set_Pwm(0,0,0,0,0);
				if(Proc_Flag==6)		TIM8_SERVO_Init(9999,168-1);					//��·���
				if(Proc_Flag==7)																					//���ƶ��
				{
					if(servo_direction[0]==0&&Servo_Count[0]<2500) Servo_Count[0]=Servo_Count[0]+5;
				 if(servo_direction[0]==0&&Servo_Count[0]>=2500) servo_direction[0]=1;
				 if(Servo_Count[0]>500&&servo_direction[0]==1)  Servo_Count[0]=Servo_Count[0]-5;
				 if(Servo_Count[0]<=500&&servo_direction[0]==1)  Servo_Count[0]=500,servo_direction[0] = 2;
				 TIM12->CCR2=Servo_Count[0];
				 
				}
				if(Proc_Flag==8)
				{
					if(servo_direction[0]!=2)					Servo_Count[0]=500,TIM12->CCR2=Servo_Count[0];
					if(servo_direction[1]==0&&Servo_Count[1]<2500) Servo_Count[1]=Servo_Count[1]+5;
				 if(servo_direction[1]==0&&Servo_Count[1]>=2500) servo_direction[1]=1;
				 if(Servo_Count[1]>500&&servo_direction[1]==1)  Servo_Count[1]=Servo_Count[1]-5;
				 if(Servo_Count[1]<=500&&servo_direction[1]==1)  Servo_Count[1]=500,servo_direction[1] = 2;
					TIM12->CCR1=Servo_Count[1];
				}
				if(Proc_Flag==9)
				{
					if(servo_direction[1]!=2)					Servo_Count[1]=500,TIM12->CCR1=Servo_Count[1];
					if(servo_direction[2]==0&&Servo_Count[2]<2500) Servo_Count[2]=Servo_Count[2]+5;
				 if(servo_direction[2]==0&&Servo_Count[2]>=2500) servo_direction[2]=1;
				 if(Servo_Count[2]>500&&servo_direction[2]==1)  Servo_Count[2]=Servo_Count[2]-5;
				 if(Servo_Count[2]<=500&&servo_direction[2]==1)  Servo_Count[2]=500,servo_direction[2] = 2;
					TIM8->CCR4=Servo_Count[2];
				}
				if(Proc_Flag==10)
				{
					if(servo_direction[2]!=2)					Servo_Count[2]=500,TIM8->CCR4=Servo_Count[2];
					if(servo_direction[3]==0&&Servo_Count[3]<2500) Servo_Count[3]=Servo_Count[3]+5;
				 if(servo_direction[3]==0&&Servo_Count[3]>=2500) servo_direction[3]=1;
				 if(Servo_Count[3]>500&&servo_direction[3]==1)  Servo_Count[3]=Servo_Count[3]-5;
				 if(Servo_Count[3]<=500&&servo_direction[3]==1)  Servo_Count[3]=500,servo_direction[3] = 2;
					TIM8->CCR3=Servo_Count[3];
				}
				if(Proc_Flag==11)
				{
					if(servo_direction[3]!=2)					Servo_Count[3]=500,TIM8->CCR3=Servo_Count[3];
					if(servo_direction[4]==0&&Servo_Count[4]<2500) Servo_Count[4]=Servo_Count[4]+5;
				 if(servo_direction[4]==0&&Servo_Count[4]>=2500) servo_direction[4]=1;
				 if(Servo_Count[4]>500&&servo_direction[4]==1)  Servo_Count[4]=Servo_Count[4]-5;
				 if(Servo_Count[4]<=500&&servo_direction[4]==1)  Servo_Count[4]=500,servo_direction[4] = 2;
					TIM8->CCR2=Servo_Count[4];
				}
				if(Proc_Flag==12)
				{
					if(servo_direction[4]!=2)					Servo_Count[4]=500,TIM8->CCR2=Servo_Count[4];
					if(servo_direction[5]==0&&Servo_Count[5]<2500) Servo_Count[5]=Servo_Count[5]+5;
				 if(servo_direction[5]==0&&Servo_Count[5]>=2500) servo_direction[5]=1;
				 if(Servo_Count[5]>500&&servo_direction[5]==1)  Servo_Count[5]=Servo_Count[5]-5;
				 if(Servo_Count[5]<=500&&servo_direction[5]==1)  Servo_Count[5]=500,servo_direction[5] = 2;
					TIM8->CCR1=Servo_Count[5];
				}
				
				if(Proc_Flag==13)																	//
				{
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					 TIM8->CCR1=Servo_Count[5];
					 TIM8->CCR2=Servo_Count[4];
					 TIM8->CCR3=Servo_Count[3];
					 TIM8->CCR4=Servo_Count[2];
					 TIM12->CCR1=Servo_Count[1];
					 TIM12->CCR2=Servo_Count[0];
				}
				if(Proc_Flag==14)																	//���������1s��һ��
				{
					if((Buzzer_count1/100)%2)			Buzzer = 1;
					else													Buzzer = 0;
				}
				if(Proc_Flag==15)			Buzzer = 0;
//				if(Proc_Flag==17)																	//��APP����WHEELTEC
//				{
//					if(uart2_send_flag==1)
//					{
//						USART2_Return();
//						uart2_send_flag = 0;
//						app_count = 0;
//					}
//				}
				if(Proc_Flag==19)
				{
					if(uart3_send_flag==1)
					{
						USART3_Return();
						uart3_send_flag = 0;
						message_count = 0;
					}
				}
			}
//************************************�Լ����************************************
	}  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//�������ת����
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//Forward and reverse control of motor
	//�������ת����	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//�������ת����	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//�������ת����
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//�������
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬������ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ��������ƣ�1����������0����
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm = Pwm + Velocity_KP*(Bias-Last_bias) + Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	
	//���PWM��־λ����λΪ1ʱ������Ҫ���PWM
	if( start_clear ) 
	{
		//PWM�𽥵ݼ��ķ�ʽ���������С�����ڵ���ͷŶ������΢�ƶ���Ӱ��
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		//�������ϣ����Ǳ�־λ��4������ֱ���4��bit��ʾ
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}
	
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm = Pwm + Velocity_KP*(Bias-Last_bias) + Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<1;
		else clear_state &= ~(1<<1);
	}
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm = Pwm + Velocity_KP*(Bias-Last_bias) + Velocity_KI*Bias;
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	
	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car) Pwm = 0;
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<2;
		else clear_state &= ~(1<<2);
	}
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm = Pwm+ Velocity_KP*(Bias-Last_bias) + Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	
	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car || Car_Mode==Omni_Car ) Pwm = 0;
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<3;
		else clear_state &= ~(1<<3);
		
		//4������������ϣ���ر��������
		if( (clear_state&0xff)==0x0f ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550) //The omnidirectional wheel moving trolley can move laterally //ȫ�����˶�С�����Խ��к����ƶ�
	{
	 switch(Flag_Direction)  //Handle direction control commands //���������������
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //����޷������ָ����ת�����״̬
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //����ת  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //����ת
		 else 		               Move_Z=0;                       //stop           //ֹͣ
	 }
	}	
	else //Non-omnidirectional moving trolley //��ȫ���ƶ�С��
	{
	 switch(Flag_Direction) //Handle direction control commands //���������������
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //����ת 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //����ת	
	}
	
	//Z-axis data conversion //Z������ת��
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//�������ṹС��ת��Ϊǰ��ת��Ƕ�
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //��λת����mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//�õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
	static u8 acc_dec_filter = 0;		
	
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //����ҡ��С���ȶ���
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	if(++acc_dec_filter==15)
	{
		acc_dec_filter=0;
	  if (PS2_KEY==11)	    RC_Velocity+=5;  //To accelerate//����
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //����	
	}

	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //��PS2�ֱ�����������д���
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z������ת��
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //�������ṹС��ת��Ϊǰ��ת��Ƕ�
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //�õ�����Ŀ��ֵ�������˶�ѧ����
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���

	  //limiter //�޷�
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //��ҡ��ǰ���򡣿���ǰ�����ˡ�
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //��ҡ�����ҷ��򡣿��������ƶ�������ȫ���ֲŻ�ʹ�õ���ͨ����������С��ʹ�ø�ͨ����ΪPWM������ƶ��
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//��ҡ��ǰ��������/�Ӽ��١�
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//��ҡ�����ҷ��򡣿�����ת��
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //�������
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//�Ժ�ģң�ؿ���������д���
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z������ת��
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //�������ṹС��ת��Ϊǰ��ת��Ƕ�
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		
	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //�õ�����Ŀ��ֵ�������˶�ѧ����
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
�������ܣ������û������������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
    u8 tmp;
    //���������Ƶ��
    tmp=KEY_Scan(RATE_100_HZ,0);
		if(Check==0)
		{
    //���� �� �ֱ�ͬʱ�������ߵ��°���������Զ��س�
    if(tmp==single_click || ( Get_PS2_KEY(L2_KEY) && Get_PS2_KEY(R2_KEY) ) )
	{
		Allow_Recharge=!Allow_Recharge;
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}		

    //˫�� �� �ֱ�ͬʱ�������ߵ�ҡ��,����������
    else if(tmp==double_click || ( Get_PS2_KEY(LF_ROCKER_KEY) && Get_PS2_KEY(RT_ROCKER_KEY) )  ) 
	{
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}

    //���� �л�ҳ��
    else if(tmp==long_click )
    {
        oled_refresh_flag=1;
        oled_page++;
        if(oled_page>OLED_MAX_Page-1) oled_page=0;
    }
	}
		else if(Check==1)
		{
			if(tmp==single_click)		
			{
				Proc_Flag++;
				if(Proc_Flag==21)			
				{
					Check = 0;
					Buzzer = 0;
					Proc_Flag = 0;
					check_time_count_motor_forward=300;
					check_time_count_motor_retreat=500;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
				}
			}
			else if(tmp==double_click)
			{
				Check = 0;
				Buzzer = 0;
				Proc_Flag = 0;
				check_time_count_motor_forward=300;
				check_time_count_motor_retreat=500;
				Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
				servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
				TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
			}
		}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //��ȡ��������ԭʼ����
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	//test_num=OriginalEncoder.B;
	
	  //Decide the encoder numerical polarity according to different car models
		//���ݲ�ͬС���ͺž�����������ֵ����
		switch(Car_Mode)
		{
			case Mec_Car:case Mec_Car_V550:
			case FourWheel_Car:case FourWheel_Car_V550:
                Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Akm_Car:case Diff_Car:case Tank_Car:
				Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Omni_Car:    
				Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break;
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;
	
	if(PS2_ON_Flag)
	{
		step=0.05;
	}
	else
	{
		step=0.01;
	}
	
	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
�������ܣ���ֹ��λ��ѡ��ģʽ�����³�ʼ���������������ת����ֹͣʹ��
��ڲ�������
����  ֵ����
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//�������6�νӽ�����������ж�Ϊ�����ת���õ��ʧ��	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}

//PWM��������
void auto_pwm_clear(void)
{
	//С����̬�����ж�
	float y_accle = (float)(imu.accel.y/1671.84f);//Y����ٶ�ʵ��ֵ
	float z_accle = (float)(imu.accel.z/1671.84f);//Z����ٶ�ʵ��ֵ
	float diff;
	
	//����Y��Z���ٶ��ں�ֵ����ֵԽ�ӽ�9.8����ʾС����̬Խˮƽ
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWM�������
	if( MOTOR_A.Target !=0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f || MOTOR_D.Target != 0.0f )
	{
		start_check_flag = 1;//�����Ҫ���PWM
		wait_clear_times = 0;//��λ��ռ�ʱ
		start_clear = 0;     //��λ�����־
		
		
		//�˶�ʱб�¼������ݸ�λ
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //��Ŀ���ٶ��ɷ�0��0ʱ����ʼ��ʱ 2.5 �룬��С������б��״̬�£����pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//С����ˮƽ����ʱ�ű�����pwm����ֹС����б�����˶���������
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//�������pwm
				else clear_done_once = 1;//С����б���ϣ������������
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//�����������������Ƴ���Ϊ��pwm����һ����ֵ����10����ٴ����
	if( clear_done_once )
	{
		//С���ӽ���ˮƽ��ʱ����������������ֹС����б�����ﳵ
		if( diff > 8.8f )
		{
			//��������pwm�ٴλ��ۣ��������
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//�������pwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
}

