#ifndef __USBH_HID_JOY_H
#define __USBH_HID_JOY_H

#include "system.h"

extern HID_cb_TypeDef HID_JOY_cb;

//����ps2�ֱ��豸��ʶ��
#define Normal_PS2_VID 0x0810
#define Normal_PS2_PID 0x0001

//����ps2�ֱ���׿ģʽ�豸��ʶ��
#define _2_4G_Android_PS2_VID 0x045E
#define _2_4G_Android_PS2_PID 0x028E

//����ps2�ֱ�PCģʽ�豸��ʶ��
#define _2_4G_PC_PS2_VID 0x0079
#define _2_4G_PC_PS2_PID 0x0126

extern u8 ps2_type;

enum {
  Normal_PS2 = 1,
  _2_4G_Android_PS2,
	_2_4G_PC_PS2
};


//�ֱ�����������
typedef struct _PS2_
{
	uint8_t    LX;       //4������ҡ��ֵ 0~255
	uint8_t    LY; 
	uint8_t    RX;  
	uint8_t    RY;
	uint16_t   KeyState; //16������ֵ
}PS2_TypeDef;

extern PS2_TypeDef ps2_data;
enum {
	KEY_OFF=0,
	KEY_ON = 1
};

//PS2����ö��
enum 
{
	//ѡ�񰴼�
	SELECT_KEY		   = (1 << 0), 
	
	//����ҡ�˰��¼�ֵ
	LF_ROCKER_KEY      = (1 << 1),
	RT_ROCKER_KEY      = (1 << 2),
	
	//��ʼ����
	START_KEY          = (1 << 3),
	
	//�󰴼�����
	LF_UP              = (1 << 4), 
	LF_RIGHT           = (1 << 5),
	LF_DOWN            = (1 << 6),
	LF_LEFT            = (1 << 7),
	
	//���Ұ������ֵ
	L2_KEY             = (1 << 8),	
	R2_KEY             = (1 << 9),
	L1_KEY             = (1 << 10),  
	R1_KEY             = (1 << 11),
	
	//�Ұ�������
	RT_GREEN           = (1 << 12),
	RT_RED             = (1 << 13), 
	RT_BLUE            = (1 << 14), 
	RT_PINK            = (1 << 15),
};
#define Get_PS2_KEY(mask) ((ps2_data.KeyState & (mask)) ? KEY_ON : KEY_OFF)

//��ֱ������ַ���
//#define Get_PS2_KEY(mask)      do { if (ps2_data.KeyState & (mask)) printf(#mask "\r\n"); } while (0)


#endif
