/**
  ******************************************************************************
  * @file    usbh_core.c 
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file implements the functions for the core state machine process
  *          the enumeration and the control transfer process
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      <http://www.st.com/SLA0044>
  *
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/

#include "usbh_ioreq.h"
#include "usb_bsp.h"
#include "usbh_hcs.h"
#include "usbh_stdreq.h"
#include "usbh_core.h"
#include "usb_hcd_int.h"


/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_LIB_CORE
* @{
*/

/** @defgroup USBH_CORE 
  * @brief This file handles the basic enumeration when a device is connected 
  *          to the host.
  * @{
  */ 

/** @defgroup USBH_CORE_Private_TypesDefinitions
  * @{
  */ 
uint8_t USBH_Disconnected (USB_OTG_CORE_HANDLE *pdev); 
uint8_t USBH_Connected (USB_OTG_CORE_HANDLE *pdev); 
uint8_t USBH_SOF (USB_OTG_CORE_HANDLE *pdev); 
uint8_t USBH_PortEnabled (USB_OTG_CORE_HANDLE *pdev); 
uint8_t USBH_PortDisabled (USB_OTG_CORE_HANDLE *pdev); 

USBH_HCD_INT_cb_TypeDef USBH_HCD_INT_cb = 
{
  USBH_SOF,
  USBH_Connected, 
  USBH_Disconnected,
  USBH_PortEnabled,
  USBH_PortDisabled
};

USBH_HCD_INT_cb_TypeDef  *USBH_HCD_INT_fops = &USBH_HCD_INT_cb;

/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_Variables
  * @{
  */ 
__IO uint32_t suspend_flag = 0;
/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_FunctionPrototypes
  * @{
  */
static USBH_Status USBH_HandleEnum(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);
USBH_Status USBH_HandleControl (USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);

void USB_OTG_BSP_Resume(USB_OTG_CORE_HANDLE *pdev);                                                                 
void USB_OTG_BSP_Suspend(USB_OTG_CORE_HANDLE *pdev);

/**
  * @}
  */ 


/** @defgroup USBH_CORE_Private_Functions
  * @{
  */ 


/**
  * @brief  USBH_Connected
  *         USB Connect callback function from the Interrupt. 
  * @param  selected device
  * @retval Status
*/
/**
  * @brief  USBH_Connected
  *         USB���ӻص����������жϡ�
  * @param  selected device
  * @retval Status
*/

uint8_t USBH_Connected (USB_OTG_CORE_HANDLE *pdev)
{
  pdev->host.ConnSts = 1;
  return 0;
}

/**
  * @brief  USBH_PortEnabled
  *         USB Port Enable function. 
  * @param  selected device
  * @retval Status
*/
/**
  * @brief  USBH_PortEnabled
  *         USB�˿����ú�����
  * @param  selected device
  * @retval Status
*/

uint8_t USBH_PortEnabled (USB_OTG_CORE_HANDLE *pdev)
{
  pdev->host.PortEnabled = 1;
  return 0;
}

/**
  * @brief  USBH_PortDisabled
  *         USB Port Disable function.
  * @param  selected device
  * @retval Status
*/
/**
  * @brief  USBH_PortDisabled
  *         USB�˿ڽ��ú�����
  * @param  selected device
  * @retval Status
*/

uint8_t USBH_PortDisabled (USB_OTG_CORE_HANDLE *pdev)
{
  pdev->host.PortEnabled = 0;
  return 0;
}

/**
* @brief  USBH_Disconnected
*         USB Disconnect callback function from the Interrupt. 
* @param  selected device
* @retval Status
*/
/**
* @brief  USBH_Disconnected
*         USB�Ͽ����ӻص����������жϡ�
* @param  selected device
* @retval Status
*/

uint8_t USBH_Disconnected (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_BSP_DriveVBUS(pdev,0);
  /* Disable all interrupts. */
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTMSK, 0);
  
  /* Clear any pending interrupts. */
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, 0xFFFFFFFF);
  USB_OTG_DisableGlobalInt(pdev);
  pdev->host.ConnSts = 0;
  return 0;  
}

/**
  * @brief  USBH_SOF
  *         USB SOF callback function from the Interrupt. 
  * @param  selected device
  * @retval Status
  */
/**
  * @brief  USBH_SOF
  *         USB SOF�ص����������жϡ�
  * @param  selected device
  * @retval Status
  */

uint8_t USBH_SOF (USB_OTG_CORE_HANDLE *pdev)
{
  /* This callback could be used to implement a scheduler process */
/* �˻ص�������ʵ�ֵ��������� */
  return 0;  
}
/**
  * @brief  USBH_Init
  *         Host hardware and stack initializations 
  * @param  class_cb: Class callback structure address
  * @param  usr_cb: User callback structure address
  * @retval None
  */
/**
* @brief  USB_OTG_EnableCommonInt
*         ��ʼ��ͨ���жϣ������豸������ģʽ
* @param  pdev : ѡ�����豸
* @retval None
*/
void USBH_Init(USB_OTG_CORE_HANDLE *pdev,
               USB_OTG_CORE_ID_TypeDef coreID,
               USBH_HOST *phost,               
               USBH_Class_cb_TypeDef *class_cb, 
               USBH_Usr_cb_TypeDef *usr_cb)
{
     
	/* Hardware Init */
	//����Host���̲���1��Ӳ����ʼ��,PA11��PA12
	USB_OTG_BSP_Init(pdev);  //����Host���̲���1��Ӳ����ʼ��,PA11��PA12
  
	/* configure GPIO pin used for switching VBUS power */
	//��������Vbus��GPIO
	USB_OTG_BSP_ConfigVBUS(0);  //����Host���̲���2������������io��ʼ��
  
	/* Host de-initializations */
	//����ȥ��ʼ��				//����Host���̲���3����λhost
	USBH_DeInit(pdev, phost);
  
  /*Register class and user callbacks */
//ָ���������õĽṹ������
  phost->class_cb = class_cb; //����Host���̲���4��ָ����Ļص�����ʵ��
	
//ָ���������õĽṹ������
  phost->usr_cb = usr_cb;   //����Host���̲���5��ָ���û��Ļص�����ʵ��
    
  /* Start the USB OTG core */    
 //���������ں�
   HCD_Init(pdev , coreID); //����Host���̲���6������host�ںˡ�pdev��usb����������ݽṹ��ָ��,coreID������ָ��Ҫ��ʼ����USB�������
   
  /* Upon Init call usr call back */
  //�����û��ĳ�ʼ������   
  phost->usr_cb->Init(); //����Host���̲���7�������û�ָ���ĳ�ʼ������,�˴����õ���void USBH_USR_Init(void)
  
  /* Enable Interrupts */
  //ʹ���ж�
  USB_OTG_BSP_EnableInterrupt();//����Host���̲���8������host���ж�
}

/**
  * @brief  USBH_DeInit 
  *         Re-Initialize Host
  * @param  None 
  * @retval status: USBH_Status
  */
/**
  * @brief  USBH_DeInit 
  *         ���³�ʼ������
  * @param  None 
  * @retval status: USBH_Status
  */
USBH_Status USBH_DeInit(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  /* Software Init */
  
  phost->gState = HOST_IDLE;
  phost->gStateBkp = HOST_IDLE; 
  phost->EnumState = ENUM_IDLE;
  phost->RequestState = CMD_SEND;  
  
  phost->Control.state = CTRL_SETUP;
  phost->Control.ep0size = USB_OTG_MAX_EP0_SIZE;  
  
  phost->device_prop.address = USBH_DEVICE_ADDRESS_DEFAULT;
  phost->device_prop.speed = HPRT0_PRTSPD_FULL_SPEED;
  
  USBH_Free_Channel  (pdev, phost->Control.hc_num_in);
  USBH_Free_Channel  (pdev, phost->Control.hc_num_out);  
  return USBH_OK;
}

/**
* @brief  USBH_Process
*         USB Host core main state machine process
* @param  None 
* @retval None
*/

//HID��������
void USBH_HID_Reconnect(void)
{
	//�ر�֮ǰ������
	USBH_DeInit(&USB_OTG_Core_dev,&USB_Host);	//��λUSB HOST
	USB_OTG_StopHost(&USB_OTG_Core_dev);		//ֹͣUSBhost
	if(USB_Host.usr_cb->DeviceDisconnected)		//����,�Ž�ֹ
	{
		USB_Host.usr_cb->DeviceDisconnected(); 	//�û�����,�豸���γ���
		USBH_DeInit(&USB_OTG_Core_dev, &USB_Host);
		USB_Host.usr_cb->DeInit();//�û�����ʼ������
		USB_Host.class_cb->DeInit(&USB_OTG_Core_dev,&USB_Host.device_prop);
	}
	USB_OTG_DisableGlobalInt(&USB_OTG_Core_dev);//�ر������ж�
	//���¸�λUSB

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS,DISABLE);
	memset(&USB_OTG_Core_dev,0,sizeof(USB_OTG_CORE_HANDLE));
	memset(&USB_Host,0,sizeof(USB_Host));
	//��������USB HID�豸
	USBH_Init(&USB_OTG_Core_dev,USB_OTG_FS_CORE_ID,&USB_Host,&HID_cb,&USR_Callbacks);  
}



/**
* @brief  USBH_Process
*         USB����������״̬������
* @param  None 
* @retval None
*/
/*
�����߼���
״̬��Ĭ��Ϊ����״̬
1.��������˿ڵ��¼�״̬��������״̬�ǿ���,�ּ�⵽û���豸ʱ,��״̬������Ϊ�Ͽ�����
2.״̬��Ϊ����״̬ʱ,��״̬������Ϊ�ȴ�prt,Ȼ��λһ��usb�豸
3.״̬��Ϊ�ȴ�prt״̬ʱ,�ڼ䲻ͣ��ִ��С�ӳ�,�ȴ��˿ڱ�����,��usb�豸����,��״̬������Ϊ������״̬��
4.״̬��Ϊ������״̬ʱ,��λ�ܵ�,��λusb.����usb�豸,��ȡ��ʲô�ٶȵ��豸,ͬʱ�����йܵ�
5.
*/
#define STATE_DBUG 0

volatile static u16 kale=0;
volatile u16 int_noresponse=0;
	
//��Ҫ��������,ʶ������豸��,��ʾ�豸���γ�,Ҳ�����쳣�ж���
//һ�������޿��ж���

void USBH_Process(USB_OTG_CORE_HANDLE *pdev , USBH_HOST *phost)
{
  volatile USBH_Status status = USBH_FAIL;

	/* check for Host port events */
	/* ��������˿��¼� */
  if (((HCD_IsDeviceConnected(pdev) == 0)|| (HCD_IsPortEnabled(pdev) == 0))&& (phost->gState != HOST_IDLE)) 
  {
    if(phost->gState != HOST_DEV_DISCONNECTED) 
    {
      phost->gState = HOST_DEV_DISCONNECTED;	
    }
  }
   
  //״̬��
  switch (phost->gState)
  {
  
  case HOST_IDLE ://����״̬
	 vTaskDelay(50);
  
	#if STATE_DBUG
	printf("״̬��Ϊ����״̬\r\n");
	#endif
    if (HCD_IsDeviceConnected(pdev))  
    {
      phost->gState = HOST_WAIT_PRT_ENABLED;//��״̬����Ϊ�ȴ�prtʹ��
		vTaskDelay(100);
      HCD_ResetPort(pdev);//�ύ��λ,ʵ�����õĺ���
      phost->usr_cb->ResetDevice();//��λusb�ͻ��ص�����void USBH_USR_ResetDevice(void)
    }
    break;
    
  case HOST_WAIT_PRT_ENABLED://�ȴ�prtʹ��
	#if STATE_DBUG
	printf("״̬������PRT�ȴ�״̬��\r\n");
	#endif

    if (pdev->host.PortEnabled == 1)
    { 
      phost->gState = HOST_DEV_ATTACHED; 
		vTaskDelay(50);
    }
    break;
      
  case HOST_DEV_ATTACHED ://�豸������
	  int_noresponse=0;
	#if STATE_DBUG
	printf("״̬�������豸����״̬��\r\n");
	#endif

    phost->usr_cb->DeviceAttached();//�����û��˺���,������õ���void USBH_USR_DeviceAttached(void)
	
	//���豸�����µ�ͨ��
    phost->Control.hc_num_out = USBH_Alloc_Channel(pdev, 0x00);
    phost->Control.hc_num_in = USBH_Alloc_Channel(pdev, 0x80);  
	
    //��λusb
    if ( HCD_ResetPort(pdev) == 0)
    {
      phost->usr_cb->ResetDevice();//�����û��˺���,������void USBH_USR_ResetDevice(void)
     
	/* Host is Now ready to start the Enumeration */
	/* ����������׼���ÿ�ʼö�� */
      phost->device_prop.speed = HCD_GetCurrentSpeed(pdev);//��ȡ�豸���ٶ�
      phost->gState = HOST_ENUMERATION;//��������״̬Ϊ׼��ö��
      phost->usr_cb->DeviceSpeedDetected(phost->device_prop.speed);//�����û��˺�������ʾ��ȡ�����豸�ٶȡ�������õ���void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
      vTaskDelay(50);
		
	/* Open Control pipes */	
	/* �򿪿��ƹܵ� */
	USBH_Open_Channel (pdev,
					 phost->Control.hc_num_in,
					 phost->device_prop.address,
					 phost->device_prop.speed,
					 EP_TYPE_CTRL,
					 phost->Control.ep0size); 
      
	/* Open Control pipes */
	/* �򿪿��ƹܵ� */
	USBH_Open_Channel (pdev,
					 phost->Control.hc_num_out,
					 phost->device_prop.address,
					 phost->device_prop.speed,
					 EP_TYPE_CTRL,
					 phost->Control.ep0size);          
    }
    break;
    
  case HOST_ENUMERATION://׼��ö��״̬ 
	  int_noresponse=0;
	#if STATE_DBUG
	printf("״̬������ö��״̬\r\n");
	#endif
	
	vTaskDelay(5);
	/* Check for enumeration status */  
	/* ���ö��״̬ */
    if ( USBH_HandleEnum(pdev , phost) == USBH_OK)//ö����� 
    { 
		/* The function shall return USBH_OK when full enumeration is complete */
		/* user callback for end of device basic enumeration */
		/* ������ö�����ʱ���ú���Ӧ����USBH_OK */
		/* �û��ص������������豸����ö�ٽ��� */
		phost->usr_cb->EnumerationDone(); //����USBH_USR_EnumerationDone
      phost->gState  = HOST_USR_INPUT;
    }
	else
	{
		enum_error++;//ö�ٴ���
		if( enum_error>50 ) NVIC_SystemReset();//ö��ʧ�ܴ���̫��,���⿨��,����ϵͳ
	}

    break;//��ö��û�����,��������ѭ�������ȴ�ö��
    	
	
  case HOST_USR_INPUT://�û�ȷ��״̬���ȴ��û�����ȷ�ϡ���ע��һ�㲻Ҫ�û�����ȷ��,�ڶ�Ӧ����ֱ�ӷ���ok
	  enum_error = 0;
	#if STATE_DBUG
	printf("״̬���������ȷ��״̬��\r\n");
	#endif
	/*The function should return user response true to move to class state */
	/* �ú���Ӧ�����û���Ӧtrue��ת����״̬ */
	//�û�����ȷ�Ϻ�,����������״̬��ʹ��ʱ��Ҫ����ȷ��,ֱ�ӷ���USBH_USR_RESP_OK�ͺ�
	//������õ����û�����UserInput()�ȴ��û�ȷ��,ʵ�ʵ��ú���ΪUSBH_USR_Status USBH_USR_UserInput(void)
    if ( phost->usr_cb->UserInput() == USBH_USR_RESP_OK)
    {
      if((phost->class_cb->Init(pdev, phost))\
        == USBH_OK) //�û�ȷ�Ϻ�,�������豸��ʼ������,����ʵ�ʵ��õĺ�����
	                //static USBH_Status USBH_HID_InterfaceInit ( USB_OTG_CORE_HANDLE *pdev, void *phost)
					//ע����������豸��ʵ�����������֣��ӳ����ת������ʵ��Ӧ�ò㡣����HID_Machine.cb = Ҳ��ָ����,�������û��õ�
      {	

        phost->gState  = HOST_CLASS_REQUEST;//��ʼ���ɹ�,״̬������������ģʽ
      }     
    }   
    break;
    
  case HOST_CLASS_REQUEST://����������ģʽ
	/* process class standard control requests state machine */ 
	/* �������׼��������״̬�� */
	#if STATE_DBUG
	printf("״̬�����������豸�����ݵ�״̬��\r\n");
	#endif
  
	kale = 0;
  //����phost->class_cb->Requests(pdev, phost);���õĺ�����
  //static USBH_Status USBH_HID_ClassRequest(USB_OTG_CORE_HANDLE *pdev, void *phost)
	status = phost->class_cb->Requests(pdev, phost);
  
	//��ȡ�豸������������Ϣ,�ȴ�ִ�����
	if(status == USBH_OK)
	{
		phost->gState  = HOST_CLASS;//��������Э�����Ϣ�������,������һ��״̬
	}  
	else
	{
		USBH_ErrorHandle(phost, status);//�豸��������Ϣ����ʧ��
	}
    break;    
	
  case HOST_CLASS: //״̬��������ͽ�����,�����ǲ��ϵĶ�ȡ���ݡ��������ݵĹ���
	#if STATE_DBUG
	printf("״̬���������ݴ���״̬��\r\n");
	#endif
  
	int_noresponse = 0;
  
	/* process class state machine */
	/* ������״̬�� */
	//���ݴ�������������õ���static USBH_Status USBH_HID_Handle(USB_OTG_CORE_HANDLE *pdev , void *phost)
    status = phost->class_cb->Machine(pdev, phost);
  
	//statusĬ�Ϸ���ok,���������������Ӧ��������
    USBH_ErrorHandle(phost, status);
    break;       
    
  case HOST_CTRL_XFER:
	#if STATE_DBUG
	printf("״̬��������ʶ����״̬%d��������������\r\n",kale);
	#endif
	vTaskDelay(5);
	if(kale>150)
	{
		kale=0,phost->gState = HOST_DEV_DISCONNECTED;//��ʼ����ע��,��״̬�����븴λģʽ
	}		
	kale++;
    /* process control transfer state machine */
  /* ���̿��ƴ���״̬�� */
    USBH_HandleControl(pdev, phost);    
    break;
  
#if defined (USB_OTG_FS_LOW_PWR_MGMT_SUPPORT) || defined (USB_OTG_HS_LOW_PWR_MGMT_SUPPORT)
  case HOST_SUSPENDED:
    if (USBH_SetDeviceFeature(pdev, phost, FEATURE_SELECTOR_DEVICE, 0)==USBH_OK)
    {
      suspend_flag = 1;
      USB_OTG_BSP_Suspend(pdev);
      phost->usr_cb->UserInput();
      PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
      /* After wakeup got to HOST_WAKEUP state */
      phost->gState  = HOST_WAKEUP;
    }
    break;
      
  case HOST_WAKEUP:
    /* issue  a ClearDeviceFeature request */
    if (USBH_ClearDeviceFeature(pdev, phost, FEATURE_SELECTOR_DEVICE, 0)==USBH_OK)
    {
      phost->gState  = HOST_USR_INPUT;
    }
    break;
#endif /* USE_HOST_MODE */
  case HOST_ERROR_STATE://����״̬,��USBH_ErrorHandle��������
	#if STATE_DBUG
	printf("״̬��������󣡣�����������\r\n");
	#endif
    /* Re-Initialize Host for new Enumeration */
    USBH_DeInit(pdev, phost);
    phost->usr_cb->DeInit();
    phost->class_cb->DeInit(pdev, &phost->device_prop);
    break;
    
  case HOST_DEV_DISCONNECTED :
	#if STATE_DBUG
	printf("״̬������Ͽ�����״̬\r\n");
	#endif
  
	usb_ps2_ready=0;//�豸�γ�

	USBH_HID_Reconnect();//���³�ʼ������
  
    /* Manage User disconnect operations*/
	/* �����û��Ͽ����ӵĲ���,�����û��˻ص�����*/
//    phost->usr_cb->DeviceDisconnected();//�豸���γ���
//    
//    /* Re-Initialize Host for new Enumeration */
//	/* Ϊ�µ�ö�����³�ʼ������ */
//    USBH_DeInit(pdev, phost);
//    phost->usr_cb->DeInit(); //�û�����ʼ������ִ�� ���� �û�����ʼ��  USBH_USR_DeInit(�û���)
//    phost->class_cb->DeInit(pdev, &phost->device_prop);//���� USBH_HID_InterfaceDeInit(�����)
//    USBH_DeAllocate_AllChannel(pdev);  //�ر����йܵ� 
//    phost->gState = HOST_IDLE;
//   
//	//IO�ڹر�
//	USB_OTG_DeInit();//IO�ڷ���ʼ��
//	
//	//�жϹر�
//	USB_OTG_BSP_DisableInterrupt();
//  
////    /* Re-Initialize Host for new Enumeration */
////  /* Ϊ�µ�ö�����³�ʼ������ */
////    HCD_Init(pdev,USB_OTG_FS_CORE_ID);
//	
//	vTaskDelay(500);
//	
//	#if STATE_DBUG
//	printf("�ȴ����³�ʼ��");
//	#endif
//	//���³�ʼ������
//	USBH_Init(&USB_OTG_Core_dev,USB_OTG_FS_CORE_ID,&USB_Host,&HID_cb,&USR_Callbacks);
//	#if STATE_DBUG
//	printf("������������һ��HOST����\r\n");
//	#endif
	
	
    break;
    
  default :
    break;
  }

}


/**
  * @brief  USBH_ErrorHandle 
  *         This function handles the Error on Host side.
  * @param  errType : Type of Error or Busy/OK state
  * @retval None
  */
/**
  * @brief USBH_ErrorHandle 
  * �˺������������˵Ĵ���
  * @param errType �� �������ͻ�æ/ȷ��״̬
  * ��
  */
void USBH_ErrorHandle(USBH_HOST *phost, USBH_Status errType)
{
  /* Error unrecovered or not supported device speed */
	/* δ�޸��Ĵ����֧�ֵ��豸�ٶ� */
  if ( (errType == USBH_ERROR_SPEED_UNKNOWN) ||
       (errType == USBH_UNRECOVERED_ERROR) )
  {
    phost->usr_cb->UnrecoveredError(); 
    phost->gState = HOST_ERROR_STATE;   
  }  
  /* USB host restart requested from application layer */
 /* Ӧ�ò��������� USB ���� */ 
  else if(errType == USBH_APPLY_DEINIT)
  {
    phost->gState = HOST_ERROR_STATE;  
	/* user callback for initialization */
	/* �û���ʼ���ص� */
    phost->usr_cb->Init();
  } 
}


/**
  * @brief  USBH_HandleEnum 
  *         This function includes the complete enumeration process
  * @param  pdev: Selected device
  * @retval USBH_Status
  */
/**
  * @brief USBH_HandleEnum 
  * �ú�������������ö�ٹ���
  * @param pdev��ѡ�е��豸
  * @retval USBH_Status
  */
#define STATE_DBUG_Enum 0

static USBH_Status USBH_HandleEnum(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  USBH_Status Status = USBH_BUSY;  
  uint8_t Local_Buffer[64];
  
  switch (phost->EnumState)
  {
  case ENUM_IDLE:  
    /* Get Device Desc for only 1st 8 bytes : To get EP0 MaxPacketSize */
    if ( USBH_Get_DevDesc(pdev , phost, 8) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("��1��ö��ok\r\n");
		#endif
		
      phost->Control.ep0size = phost->device_prop.Dev_Desc.bMaxPacketSize;
      
      phost->EnumState = ENUM_GET_FULL_DEV_DESC;
      
      /* modify control channels configuration for MaxPacket size */
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_out,
                           0,
                           0,
                           0,
                           phost->Control.ep0size);
      
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_in,
                           0,
                           0,
                           0,
                           phost->Control.ep0size);      
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!��1��ö�ٲ�OK\r\n");
		#endif
	}
    break;
    
  case ENUM_GET_FULL_DEV_DESC:  
    /* Get FULL Device Desc  */
    if ( USBH_Get_DevDesc(pdev, phost, USB_DEVICE_DESC_SIZE)\
      == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("��2��ö��ok\r\n");
		#endif
      /* user callback for device descriptor available */
      phost->usr_cb->DeviceDescAvailable(&phost->device_prop.Dev_Desc);      
      phost->EnumState = ENUM_SET_ADDR;
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!��2��ö�ٲ�OK\r\n");
		#endif
	}
    break;
   
  case ENUM_SET_ADDR: 
    /* set address */
    if ( USBH_SetAddress(pdev, phost, USBH_DEVICE_ADDRESS) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("��3��ö��ok\r\n");
		#endif
      USB_OTG_BSP_mDelay(2);
      phost->device_prop.address = USBH_DEVICE_ADDRESS;
      
      /* user callback for device address assigned */
      phost->usr_cb->DeviceAddressAssigned();
      phost->EnumState = ENUM_GET_CFG_DESC;
      
      /* modify control channels to update device address */
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_in,
                           phost->device_prop.address,
                           0,
                           0,
                           0);
      
      USBH_Modify_Channel (pdev,
                           phost->Control.hc_num_out,
                           phost->device_prop.address,
                           0,
                           0,
                           0);         
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!��3��ö�ٲ�OK\r\n");
		#endif
	}
    break;
    
  case ENUM_GET_CFG_DESC:  
    /* get standard configuration descriptor */
    if ( USBH_Get_CfgDesc(pdev, 
                          phost,
                          USB_CONFIGURATION_DESC_SIZE) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("��4��ö��ok\r\n");
		#endif
      /* before getting full config descriptor, check if it does not exceed 
      buffer size allocated to config descriptor USBH_MAX_DATA_BUFFER
      in the usbh_conf.h*/
      if (phost->device_prop.Cfg_Desc.wTotalLength <= USBH_MAX_DATA_BUFFER)
      {
        phost->EnumState = ENUM_GET_FULL_CFG_DESC;
      }
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!��4��ö�ٲ�OK\r\n");
		#endif
	}
    break;
    
  case ENUM_GET_FULL_CFG_DESC:  
    /* get FULL config descriptor (config, interface, endpoints) */
    if (USBH_Get_CfgDesc(pdev, 
                         phost,
                         phost->device_prop.Cfg_Desc.wTotalLength) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("��5��ö��ok\r\n");
		#endif
      /* User callback for configuration descriptors available */
      phost->usr_cb->ConfigurationDescAvailable(&phost->device_prop.Cfg_Desc,
                                                      phost->device_prop.Itf_Desc,
                                                      phost->device_prop.Ep_Desc[0]);
      
      phost->EnumState = ENUM_GET_MFC_STRING_DESC;
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!��5��ö�ٲ�OK\r\n");
		#endif
	}
    break;
    
  case ENUM_GET_MFC_STRING_DESC:  
    if (phost->device_prop.Dev_Desc.iManufacturer != 0)
    { /* Check that Manufacturer String is available */
      
      if ( USBH_Get_StringDesc(pdev,
                               phost,
                               phost->device_prop.Dev_Desc.iManufacturer, 
                               Local_Buffer , 
                               0xff) == USBH_OK)
      {
		#if STATE_DBUG_Enum
		printf("��6��ö��ok\r\n");
		#endif
        /* User callback for Manufacturing string */
        phost->usr_cb->ManufacturerString(Local_Buffer);
        phost->EnumState = ENUM_GET_PRODUCT_STRING_DESC;
      }
    }
    else
    {
		#if STATE_DBUG_Enum
		printf("��6��ö��ok\r\n");
		#endif
      phost->usr_cb->ManufacturerString("N/A");      
      phost->EnumState = ENUM_GET_PRODUCT_STRING_DESC;
    }
    break;
    
  case ENUM_GET_PRODUCT_STRING_DESC:   
    if (phost->device_prop.Dev_Desc.iProduct != 0)
    { /* Check that Product string is available */
      if ( USBH_Get_StringDesc(pdev,
                               phost,
                               phost->device_prop.Dev_Desc.iProduct, 
                               Local_Buffer, 
                               0xff) == USBH_OK)
      {
		#if STATE_DBUG_Enum
		printf("��7��ö��ok\r\n");
		#endif
        /* User callback for Product string */
        phost->usr_cb->ProductString(Local_Buffer);
        phost->EnumState = ENUM_GET_SERIALNUM_STRING_DESC;
      }
    }
    else
    {
		#if STATE_DBUG_Enum
		printf("��7��ö��ok\r\n");
		#endif
      phost->usr_cb->ProductString("N/A");
      phost->EnumState = ENUM_GET_SERIALNUM_STRING_DESC;
    } 
    break;
    
  case ENUM_GET_SERIALNUM_STRING_DESC:   
    if (phost->device_prop.Dev_Desc.iSerialNumber != 0)
    { /* Check that Serial number string is available */    
      if ( USBH_Get_StringDesc(pdev, 
                               phost,
                               phost->device_prop.Dev_Desc.iSerialNumber, 
                               Local_Buffer, 
                               0xff) == USBH_OK)
      {
		#if STATE_DBUG_Enum
		printf("��8��ö��ok\r\n");
		#endif
        /* User callback for Serial number string */
        phost->usr_cb->SerialNumString(Local_Buffer);
        phost->EnumState = ENUM_SET_CONFIGURATION;
      }
    }
    else
    {
		#if STATE_DBUG_Enum
		printf("��8��ö��ok\r\n");
		#endif
      phost->usr_cb->SerialNumString("N/A");      
      phost->EnumState = ENUM_SET_CONFIGURATION;
    }  
    break;
      
  case ENUM_SET_CONFIGURATION:
    /* set configuration  (default config) */
    if (USBH_SetCfg(pdev, 
                    phost,
                    phost->device_prop.Cfg_Desc.bConfigurationValue) == USBH_OK)
    {
		#if STATE_DBUG_Enum
		printf("��9��ö��ok\r\n");
		#endif
      phost->EnumState = ENUM_DEV_CONFIGURED;
    }
	else
	{
		#if STATE_DBUG_Enum
		printf("!!!!!!!!!!!!��9��ö�ٲ�OK\r\n");
		#endif
	}
    break;

    
  case ENUM_DEV_CONFIGURED:
		#if STATE_DBUG_Enum
		printf("��10��ö��ok\r\n");
		#endif
    /* user callback for enumeration done */
    Status = USBH_OK;
    break;
    
  default:
    break;
  }  
  return Status;
}


/**
  * @brief  USBH_HandleControl
  *         Handles the USB control transfer state machine
  * @param  pdev: Selected device
  * @retval Status
  */
/**
  * USBH_HandleControl
  * ���� USB ���ƴ���״̬��
  * @param pdev��ѡ�е��豸
  * @retval ״̬
  */
USBH_Status USBH_HandleControl (USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  uint8_t direction;  
  static uint16_t timeout = 0;
  USBH_Status status = USBH_OK;
  URB_STATE URB_Status = URB_IDLE;
  
  phost->Control.status = CTRL_START;

  
  switch (phost->Control.state)
  {
  case CTRL_SETUP:
    /* send a SETUP packet */
    USBH_CtlSendSetup     (pdev, 
	                   phost->Control.setup.d8 , 
	                   phost->Control.hc_num_out);  
    phost->Control.state = CTRL_SETUP_WAIT;  
    break; 
    
  case CTRL_SETUP_WAIT:
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_out); 
    /* case SETUP packet sent successfully */
    if(URB_Status == URB_DONE)
    { 
      direction = (phost->Control.setup.b.bmRequestType & USB_REQ_DIR_MASK);
      
      /* check if there is a data stage */
      if (phost->Control.setup.b.wLength.w != 0 )
      {        
        timeout = DATA_STAGE_TIMEOUT;
        if (direction == USB_D2H)
        {
          /* Data Direction is IN */
          phost->Control.state = CTRL_DATA_IN;
        }
        else
        {
          /* Data Direction is OUT */
          phost->Control.state = CTRL_DATA_OUT;
        } 
      }
      /* No DATA stage */
      else
      {
        timeout = NODATA_STAGE_TIMEOUT;
        
        /* If there is No Data Transfer Stage */
        if (direction == USB_D2H)
        {
          /* Data Direction is IN */
          phost->Control.state = CTRL_STATUS_OUT;
        }
        else
        {
          /* Data Direction is OUT */
          phost->Control.state = CTRL_STATUS_IN;
        } 
      }          
      /* Set the delay timer to enable timeout for data stage completion */
      phost->Control.timer = HCD_GetCurrentFrame(pdev);
    }
    else if(URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;     
      phost->Control.status = CTRL_XACTERR;
    }    
    break;
    
  case CTRL_DATA_IN:  
    /* Issue an IN token */ 
    USBH_CtlReceiveData(pdev,
                        phost->Control.buff, 
                        phost->Control.length,
                        phost->Control.hc_num_in);
 
    phost->Control.state = CTRL_DATA_IN_WAIT;
    break;    
    
  case CTRL_DATA_IN_WAIT:
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_in); 
    
    /* check is DATA packet transferred successfully */
    if  (URB_Status == URB_DONE)
    { 
      phost->Control.state = CTRL_STATUS_OUT;
    }
   
    /* manage error cases*/
    if  (URB_Status == URB_STALL) 
    { 
      /* In stall case, return to previous machine state*/
      phost->gState =   phost->gStateBkp;
      phost->Control.state = CTRL_STALLED;  
    }   
    else if (URB_Status == URB_ERROR)
    {
      /* Device error */
      phost->Control.state = CTRL_ERROR;    
    }
    else if ((HCD_GetCurrentFrame(pdev)- phost->Control.timer) > timeout)
    {
      /* timeout for IN transfer */
      phost->Control.state = CTRL_ERROR; 
    }   
    break;
    
  case CTRL_DATA_OUT:
    /* Start DATA out transfer (only one DATA packet)*/
    pdev->host.hc[phost->Control.hc_num_out].toggle_out = 1; 
        
    USBH_CtlSendData (pdev,
                      phost->Control.buff, 
                      phost->Control.length , 
                      phost->Control.hc_num_out);
    
    phost->Control.state = CTRL_DATA_OUT_WAIT;
    break;
    
  case CTRL_DATA_OUT_WAIT:
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_out);     
    if  (URB_Status == URB_DONE)
    { /* If the Setup Pkt is sent successful, then change the state */
      phost->Control.state = CTRL_STATUS_IN;
    }
    
    /* handle error cases */
    else if  (URB_Status == URB_STALL) 
    { 
      /* In stall case, return to previous machine state*/
      phost->gState =   phost->gStateBkp;
      phost->Control.state = CTRL_STALLED;  
    } 
    else if  (URB_Status == URB_NOTREADY)
    { 
      /* Nack received from device */
      phost->Control.state = CTRL_DATA_OUT;
    }    
    else if (URB_Status == URB_ERROR)
    {
      /* device error */
      phost->Control.state = CTRL_ERROR;      
    } 
    break;
    
    
  case CTRL_STATUS_IN:
    /* Send 0 bytes out packet */
    USBH_CtlReceiveData (pdev,
                         0,
                         0,
                         phost->Control.hc_num_in);
    
    phost->Control.state = CTRL_STATUS_IN_WAIT;
    
    break;
    
  case CTRL_STATUS_IN_WAIT:
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_in); 
    
    if  ( URB_Status == URB_DONE)
    { /* Control transfers completed, Exit the State Machine */
      phost->gState =   phost->gStateBkp;
      phost->Control.state = CTRL_COMPLETE;
    }
    
    else if (URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;  
    }
    
    else if((HCD_GetCurrentFrame(pdev)\
      - phost->Control.timer) > timeout)
    {
      phost->Control.state = CTRL_ERROR; 
    }
     else if(URB_Status == URB_STALL)
    {
      /* Control transfers completed, Exit the State Machine */
      phost->gState =   phost->gStateBkp;
      phost->Control.state = CTRL_STALLED;
      status = USBH_NOT_SUPPORTED;
    }
    break;
    
  case CTRL_STATUS_OUT:
    pdev->host.hc[phost->Control.hc_num_out].toggle_out ^= 1; 
    USBH_CtlSendData (pdev,
                      0,
                      0,
                      phost->Control.hc_num_out);
    
    phost->Control.state = CTRL_STATUS_OUT_WAIT;
    break;
    
  case CTRL_STATUS_OUT_WAIT: 
    
    URB_Status = HCD_GetURB_State(pdev , phost->Control.hc_num_out);  
    if  (URB_Status == URB_DONE)
    { 
      phost->gState =   phost->gStateBkp; 
      phost->Control.state = CTRL_COMPLETE; 
    }
    else if  (URB_Status == URB_NOTREADY)
    { 
      phost->Control.state = CTRL_STATUS_OUT;
    }      
    else if (URB_Status == URB_ERROR)
    {
      phost->Control.state = CTRL_ERROR;      
    }
    break;
    
  case CTRL_ERROR:
    /* 
    After a halt condition is encountered or an error is detected by the 
    host, a control endpoint is allowed to recover by accepting the next Setup 
    PID; i.e., recovery actions via some other pipe are not required for control
    endpoints. For the Default Control Pipe, a device reset will ultimately be 
    required to clear the halt or error condition if the next Setup PID is not 
    accepted.
    */
  /*����ֹͣ������������⵽����� 
    ������⵽����󣬿��ƶ˵��ͨ��������һ������ 
    Ҳ����˵�����ƶ˵㲻��Ҫͨ�������ܵ����лָ�������
    �����ƶ˵㲻��Ҫͨ�������ܵ����лָ�����������Ĭ�Ͽ��ƹܵ���������Ҫ�����豸 
    �����������һ������ PID����������Ҫ�����豸�����ֹͣ�����״̬��*/
    if (++ phost->Control.errorcount <= USBH_MAX_ERROR_COUNT)
    {
      /* Do the transmission again, starting from SETUP Packet */
      phost->Control.state = CTRL_SETUP; 
    }
    else
    {
      phost->Control.status = CTRL_FAIL;
      phost->gState =   phost->gStateBkp;
      
      status = USBH_FAIL;
    }
    break;
    
  default:
    break;
  }
  return status;
}

/**
  * @brief  USB_OTG_BSP_Resume
  *         Handles the USB Resume from Suspend Mode
  * @param  pdev: Selected device
  * @retval Status
  */
/**
  * USB_OTG_BSP_Resume
  * ���� USB �ӹ���ģʽ�ָ�
  * @param pdev��ѡ�е��豸
  * @retval ״̬
  */
void USB_OTG_BSP_Resume(USB_OTG_CORE_HANDLE *pdev)                                                                 
{
  USB_OTG_HPRT0_TypeDef    hprt0;
  USB_OTG_PCGCCTL_TypeDef  power;
  hprt0.d32  = 0;
  /*  switch-off the clocks */
  power.d32 = 0;
  power.b.stoppclk = 1;
  USB_OTG_MODIFY_REG32(pdev->regs.PCGCCTL,power.d32, 0 );  
  
  power.b.gatehclk = 1;
  USB_OTG_MODIFY_REG32(pdev->regs.PCGCCTL,power.d32, 0);
  
  hprt0.d32  = 0;   
  hprt0.d32  = USB_OTG_ReadHPRT0(pdev);
  hprt0.b.prtsusp = 0;
  hprt0.b.prtres = 1;
  USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32);
  USB_OTG_BSP_mDelay (20);
  hprt0.b.prtres = 0;
  USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32); 
}

/**
  * @brief  USB_OTG_BSP_Suspend
  *         Handles the Enter USB to Suspend Mode
  * @param  pdev: Selected device
  * @retval Status
  */
/**
  * ������� USB ����ģʽ��
  * ������� USB ����ģʽ
  * @param pdev��ѡ�е��豸
  * @retval ״̬
  */
void USB_OTG_BSP_Suspend(USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_HPRT0_TypeDef    hprt0;
  USB_OTG_PCGCCTL_TypeDef  power;
  hprt0.d32  = 0;
  hprt0.d32  = USB_OTG_ReadHPRT0(pdev);
  hprt0.b.prtsusp = 1; 
  USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32);
  
  /*  switch-off the clocks */
  power.d32 = 0;
  power.b.stoppclk = 1;
  USB_OTG_MODIFY_REG32(pdev->regs.PCGCCTL, 0, power.d32);  
  
  power.b.gatehclk = 1;
  USB_OTG_MODIFY_REG32(pdev->regs.PCGCCTL, 0, power.d32);
}


/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
