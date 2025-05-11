#include "AutoRecharge.h"

// Enable navigation to find charging pile
//�Ƿ�������Ѱ�ҳ��׮
u8 nav_walk=0;
// Whether automatic charging is allowed
//�Ƿ���������Զ��س�
u8 Allow_Recharge=0;
// Whether the robot is charging, and whether the recharge equipment receives infrared signals
//�������Ƿ��ڳ�磬�س�װ���Ƿ���յ������ź�
u8 Charging, RED_STATE;
float Charging_Current=0;
float Recharge_Red_Move_X, Recharge_Red_Move_Y, Recharge_Red_Move_Z; 
float Recharge_UP_Move_X, Recharge_UP_Move_Y, Recharge_UP_Move_Z; 
float Red_Docker_X=-0.15, Red_Docker_Y=0, Red_Docker_Z=0.1; 
u8 L_A,L_B,R_B,R_A;
void CAN_Send_AutoRecharge(void)
{
	u8 CAN_SENT[8];
	
	//Ԥ��λ
	CAN_SENT[0]=0;
	
	//Set the speed of the infrared interconnection, unit m/s
	//���ú���Խӵ��ٶȴ�С����λmm/s
	CAN_SENT[1]=((short)(Red_Docker_X*1000))>>8;
	CAN_SENT[2]=((short)(Red_Docker_X*1000));
	CAN_SENT[3]=((short)(Red_Docker_Y*1000))>>8;
	CAN_SENT[4]=((short)(Red_Docker_Y*1000));
	CAN_SENT[5]=((short)(Red_Docker_Z*1000))>>8;
	CAN_SENT[6]=((short)(Red_Docker_Z*1000));
	
	CAN1_Send_Num(0x105,CAN_SENT);
}

