#include "sys.h"  

//���ڵ��Ե�ϵͳ����
DEBUG_t debug;

//����汾����,���ذ汾��Ϣ����
const uint8_t* getSW_Ver(SOFTWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_00: return "V1.00";
		//���������汾.
	}
	
	return "unknown";
}

const uint8_t* getHW_Ver(HARDWARE_VERSION ver)
{
	switch( ver )
	{
		case V1_0: return "V1.0";
		case V1_1: return "V1.1";
		//���������汾.
	}
	
	return "unknown";
}


//ϵͳ��������ʼ��
//������Ӳ���汾��Ϣ.Ӳ���汾��Ӳ����ʼ�������ȷ��.
void SYS_VAL_t_Init(SYS_VAL_t* p)
{
	//��ǰ����汾
	p->SoftWare_Ver = V1_00;
	
	p->HardWare_Ver = HW_NONE;//��Ӳ����ʼ����ȷ��
	
	p->Time_count = 0;
	p->HardWare_charger = 0;
	p->SecurityLevel = 1; //ϵͳ��ȫ�ȼ�,Ĭ������1.�������ʧ����
	p->LED_delay = 1; //ϵͳ״ָ̬ʾ����˸���,��λ ms
}




//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//�ر������ж�(���ǲ�����fault��NMI�ж�)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
















