/**
  ******************************************************************************
  * @file    usbh_hid_core.c
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file is the HID Layer Handlers for USB Host HID class.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                HID Class  Description
  *          =================================================================== 
  *           This module manages the MSC class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse and Keyboard protocols
  *      
  *  @endverbatim
  *
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
#include "usbh_hid_core.h"
#include "usbh_hid_joy.h"
#include "stdio.h"
/** @addtogroup USBH_LIB
* @{
*/

/** @addtogroup USBH_CLASS
* @{
*/

/** @addtogroup USBH_HID_CLASS
* @{
*/

/** @defgroup USBH_HID_CORE 
* @brief    This file includes HID Layer Handlers for USB Host HID class.
* @{
*/ 

/** @defgroup USBH_HID_CORE_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBH_HID_CORE_Private_Defines
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBH_HID_CORE_Private_Macros
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USBH_HID_CORE_Private_Variables
* @{
*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN HID_Machine_TypeDef        HID_Machine __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN HID_Report_TypeDef         HID_Report __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_Setup_TypeDef          HID_Setup __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USBH_HIDDesc_TypeDef       HID_Desc __ALIGN_END ; 

__IO uint8_t start_toggle = 0;
/**
* @}
*/ 


/** @defgroup USBH_HID_CORE_Private_FunctionPrototypes
* @{
*/ 

static USBH_Status USBH_HID_InterfaceInit  (USB_OTG_CORE_HANDLE *pdev , 
                                            void *phost);

static void  USBH_ParseHIDDesc (USBH_HIDDesc_TypeDef *desc, uint8_t *buf);

static void USBH_HID_InterfaceDeInit  (USB_OTG_CORE_HANDLE *pdev , 
                                       void *phost);

static USBH_Status USBH_HID_Handle(USB_OTG_CORE_HANDLE *pdev , 
                                   void *phost);

static USBH_Status USBH_HID_ClassRequest(USB_OTG_CORE_HANDLE *pdev , 
                                         void *phost);

static USBH_Status USBH_Get_HID_ReportDescriptor (USB_OTG_CORE_HANDLE *pdev, 
                                                  USBH_HOST *phost,
                                                  uint16_t length);

static USBH_Status USBH_Get_HID_Descriptor (USB_OTG_CORE_HANDLE *pdev,\
                                            USBH_HOST *phost,  
                                            uint16_t length);

static USBH_Status USBH_Set_Idle (USB_OTG_CORE_HANDLE *pdev, 
                                  USBH_HOST *phost,
                                  uint8_t duration,
                                  uint8_t reportId);

static USBH_Status USBH_Set_Protocol (USB_OTG_CORE_HANDLE *pdev, 
                                      USBH_HOST *phost,
                                      uint8_t protocol);



//typedef struct _USBH_Class_cb
//{
//  USBH_Status  (*Init)      (USB_OTG_CORE_HANDLE *pdev , void *phost);
//  void         (*DeInit)    (USB_OTG_CORE_HANDLE *pdev , void *phost);
//  USBH_Status  (*Requests)  (USB_OTG_CORE_HANDLE *pdev ,void *phost);  
//  USBH_Status  (*Machine)   (USB_OTG_CORE_HANDLE *pdev, void *phost);     
//  
//} USBH_Class_cb_TypeDef;
//���ʵ��,��Ӧ��������Щ����
USBH_Class_cb_TypeDef  HID_cb = 
{
  USBH_HID_InterfaceInit,
  USBH_HID_InterfaceDeInit,
  USBH_HID_ClassRequest,
  USBH_HID_Handle
};
/**
* @}
*/ 


/** @defgroup USBH_HID_CORE_Private_Functions
* @{
*/ 

/**
* @brief  USBH_HID_InterfaceInit 
*         The function init the HID class.//�ú������ڳ�ʼ��HID��
* @param  pdev: Selected device //pdev ѡ�����豸
* @param  hdev: Selected device property //hdev:ѡ�͵��豸����
* @retval  USBH_Status :Response for USB HID driver initialization //USB HID���������ʼ������Ӧ����
*/
static USBH_Status USBH_HID_InterfaceInit ( USB_OTG_CORE_HANDLE *pdev, 
                                           void *phost)
{	
	uint8_t maxEP;
	USBH_HOST *pphost = phost;

	uint8_t num =0;
	USBH_Status status = USBH_BUSY ;
	HID_Machine.state = HID_ERROR;

	//ָ��ps2�ֱ�ʵ��
	HID_Machine.cb = &HID_JOY_cb;
	  
	//��ȷ�����豸�󣬿�ʼ��ʼ��HID_Machine�ṹ�塣HID_Machine��Ӧ�˾�����豸
    HID_Machine.state     = HID_IDLE;
    HID_Machine.ctl_state = HID_REQ_IDLE; 
    HID_Machine.ep_addr   = pphost->device_prop.Ep_Desc[0][0].bEndpointAddress;
    HID_Machine.length    = pphost->device_prop.Ep_Desc[0][0].wMaxPacketSize;
    HID_Machine.poll      = pphost->device_prop.Ep_Desc[0][0].bInterval ;
    
//	printf("�豸���ͣ�%d\r\n",pphost->device_prop.Itf_Desc[0].bInterfaceSubClass);
//	printf("�豸���ţ�%d\r\n",pphost->device_prop.Itf_Desc[0].bInterfaceProtocol);
//	printf("�豸��HID��Ϣ��\r\n");
//	printf("HID_Machine.ep_addr = %d\r\n",HID_Machine.ep_addr);
//	printf("HID_Machine.length = %d\r\n",HID_Machine.length);
//	printf("HID_Machine.poll = %d\r\n",HID_Machine.poll);
	
    if (HID_Machine.poll  < HID_MIN_POLL) 
    {
       HID_Machine.poll = HID_MIN_POLL;
    }

    /* Check fo available number of endpoints */
    /* Find the number of EPs in the Interface Descriptor */      
    /* Choose the lower number in order not to overrun the buffer allocated */
	/* �����õĶ˵��� */
    /* �ڽӿ��������в��Ҷ˵��� */
    /* ѡ��ϵ͵������Ա�������ѷ���Ļ����� */
    maxEP = ( (pphost->device_prop.Itf_Desc[0].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ? 
             pphost->device_prop.Itf_Desc[0].bNumEndpoints :
                 USBH_MAX_NUM_ENDPOINTS); 
    
    
    /* Decode endpoint IN and OUT address from interface descriptor */
	/* �ӽӿ��������н���˵������������ַ */
    for (num=0; num < maxEP; num++)
    {
      if(pphost->device_prop.Ep_Desc[0][num].bEndpointAddress & 0x80)
      {
        HID_Machine.HIDIntInEp = (pphost->device_prop.Ep_Desc[0][num].bEndpointAddress);
        HID_Machine.hc_num_in  =\
               USBH_Alloc_Channel(pdev, 
                                  pphost->device_prop.Ep_Desc[0][num].bEndpointAddress);
        
        /* Open channel for IN endpoint */
		  /* �� IN �˵��ͨ�� */
        USBH_Open_Channel  (pdev,
                            HID_Machine.hc_num_in,
                            pphost->device_prop.address,
                            pphost->device_prop.speed,
                            EP_TYPE_INTR,
                            HID_Machine.length); 
      }
      else
      {
        HID_Machine.HIDIntOutEp = (pphost->device_prop.Ep_Desc[0][num].bEndpointAddress);
        HID_Machine.hc_num_out  =\
                USBH_Alloc_Channel(pdev, 
                                   pphost->device_prop.Ep_Desc[0][num].bEndpointAddress);
        
        /* Open channel for OUT endpoint */
		  /* �� OUT �˵��ͨ�� */
        USBH_Open_Channel  (pdev,
                            HID_Machine.hc_num_out,
                            pphost->device_prop.address,
                            pphost->device_prop.speed,
                            EP_TYPE_INTR,
                            HID_Machine.length); 
      }
      
    }   
    
     start_toggle =0;
     status = USBH_OK; 
  
  return status;
  
}



/**
* @brief  USBH_HID_InterfaceDeInit 
*         The function DeInit the Host Channels used for the HID class.
* @param  pdev: Selected device
* @param  hdev: Selected device property
* @retval None
*/
/**
* @brief USBH_HID_InterfaceDeInit 
* ���� DeInit ���� HID �������ͨ����
* @param pdev��ѡ�е��豸
* @param hdev����ѡ�豸����
* ��
*/
void USBH_HID_InterfaceDeInit ( USB_OTG_CORE_HANDLE *pdev,
                               void *phost)
{	
   //USBH_HOST *pphost = phost;
    
  if(HID_Machine.hc_num_in != 0x00)
  {   
    USB_OTG_HC_Halt(pdev, HID_Machine.hc_num_in);
    USBH_Free_Channel  (pdev, HID_Machine.hc_num_in);
    HID_Machine.hc_num_in = 0;     /* Reset the Channel as Free */  
  }
  
  if(HID_Machine.hc_num_out != 0x00)
  {   
    USB_OTG_HC_Halt(pdev, HID_Machine.hc_num_out);
    USBH_Free_Channel  (pdev, HID_Machine.hc_num_out);
    HID_Machine.hc_num_out = 0;     /* Reset the Channel as Free */  
  }
 
  start_toggle = 0;
}

/**
* @brief  USBH_HID_ClassRequest 
*         The function is responsible for handling HID Class requests
*         for HID class.
* @param  pdev: Selected device
* @param  hdev: Selected device property
* @retval  USBH_Status :Response for USB Set Protocol request
*/
/*
brief  USBH_HID_ClassRequest 
       �ú���������HID���HID������
param  pdev: ѡ�����豸
param  hdev: ѡ�����豸����
retval  USBH_Status : USB����Э���������Ӧ
*/
static USBH_Status USBH_HID_ClassRequest(USB_OTG_CORE_HANDLE *pdev , 
                                         void *phost)
{   
	USBH_HOST *pphost = phost;

	USBH_Status status         = USBH_BUSY;
	USBH_Status classReqStatus = USBH_BUSY;
  
  
	/* Switch HID state machine */
	/* �л�HID״̬�� */
  switch (HID_Machine.ctl_state)
  {
	  case HID_IDLE:  //����
	  case HID_REQ_GET_HID_DESC://��ȡ�豸������״̬
    
    /* Get HID Desc */ 
	/* ��ȡHID������ */
    if (USBH_Get_HID_Descriptor (pdev, pphost, USB_HID_DESC_SIZE)== USBH_OK)
    {
		USBH_ParseHIDDesc(&HID_Desc, pdev->host.Rx_Buffer);//�����豸������
		HID_Machine.ctl_state = HID_REQ_GET_REPORT_DESC;//״̬��������һ��״̬
    }
    break;     
	
  case HID_REQ_GET_REPORT_DESC://
	/* Get Report Desc */ 
	/* ��ȡ���������� */
    if (USBH_Get_HID_ReportDescriptor(pdev , pphost, HID_Desc.wItemLength) == USBH_OK)
    {
      HID_Machine.ctl_state = HID_REQ_SET_IDLE;
    }
    
    break;
    
  case HID_REQ_SET_IDLE:
    
	//����usb hostΪ����״̬
    classReqStatus = USBH_Set_Idle (pdev, pphost, 0, 0);
    
	/* set Idle */
	/* ���ÿ���״̬ */
    if (classReqStatus == USBH_OK)
    {
      HID_Machine.ctl_state = HID_REQ_SET_PROTOCOL;  
    }
    else if(classReqStatus == USBH_NOT_SUPPORTED) 
    {
      HID_Machine.ctl_state = HID_REQ_SET_PROTOCOL;        
    } 
    break; 
    
  case HID_REQ_SET_PROTOCOL:
	/* set protocol */
	/* ����Э�� */
    if (USBH_Set_Protocol (pdev ,pphost, 0) == USBH_OK)
    {
      HID_Machine.ctl_state = HID_REQ_IDLE;
      
      /* all requests performed*/
		/* ����������ִ�� */
      status = USBH_OK; 
    } 
    break;
    
  default:
    break;
  }
  
  return status; 
}


/**
* @brief  USBH_HID_Handle 
*         The function is for managing state machine for HID data transfers 
* @param  pdev: Selected device
* @param  hdev: Selected device property
* @retval USBH_Status
*/
/*brief  USBH_HID_Handle 
      �ú������ڹ���HID���ݴ����״̬��
 @param  pdev: ѡ�����豸
 @param  hdev: ѡ�����豸����
@retval USBH_Status
*/

static USBH_Status USBH_HID_Handle(USB_OTG_CORE_HANDLE *pdev , 
                                   void   *phost)
{
  USBH_HOST *pphost = phost;
  USBH_Status status = USBH_OK;
  
  switch (HID_Machine.state)
  {
    
	case HID_IDLE://����״̬
		
	//�ں���USBH_HID_InterfaceInit������Initִ�еĶ���
	HID_Machine.cb->Init(); //������õ���static void  MOUSE_Init (void) �� static void  Keybd_Init ( void)
	HID_Machine.state = HID_SYNC;//������һ��״̬
    
  case HID_SYNC:
	/* Sync with start of Even Frame */
	/* ��ż��֡�Ŀ�ʼͬ�� */
    if(USB_OTG_IsEvenFrame(pdev) == TRUE)//����ָ���豸������
    {
      HID_Machine.state = HID_GET_DATA; //״̬�����Ϊ��ȡ������
    }

    break;
   
	//�ڱ��ΪHID_GET_DATA�󣬽���ʼ���ϵض�ȡ���ݺʹ������ݣ�״̬����HID_POLL��HID_GET_DATA֮��ѭ��
  case HID_GET_DATA://��ȡ������
	
	//��Ӧ��������
    USBH_InterruptReceiveData(pdev, 
                              HID_Machine.buff,
                              HID_Machine.length,
                              HID_Machine.hc_num_in);
    start_toggle = 1;
    
    HID_Machine.state = HID_POLL;//��ѯ״̬
    HID_Machine.timer = HCD_GetCurrentFrame(pdev);//�������ݰ�֡���
    break;
    
  //��ѯ״̬,��ȡ���ݹ���
  case HID_POLL:
  
    if(( HCD_GetCurrentFrame(pdev) - HID_Machine.timer) >= HID_Machine.poll)
    {
      HID_Machine.state = HID_GET_DATA;
    }
	
	//��ȡ��������,�����ݴ���
    else if(HCD_GetURB_State(pdev , HID_Machine.hc_num_in) == URB_DONE)
    {
      if(start_toggle == 1) /* handle data once */
      {
        start_toggle = 0;
		//���� JOY_Decode
        HID_Machine.cb->Decode(HID_Machine.buff);//�����ݽ��н���
      }
    }
	
    else if(HCD_GetURB_State(pdev, HID_Machine.hc_num_in) == URB_STALL) /* IN Endpoint Stalled */ //(IN�˵���ͣ)
    {
      
      /* Issue Clear Feature on interrupt IN endpoint */ 
		/* ���ж�IN�˵��Ϸ�������������� */
      if( (USBH_ClrFeature(pdev, 
                           pphost,
                           HID_Machine.ep_addr,
                           HID_Machine.hc_num_in)) == USBH_OK)
      {
        /* Change state to issue next IN token */
		/* ��״̬����Ϊ������һ��IN���� */
        HID_Machine.state = HID_GET_DATA;
        
      }
      
    }      
    break;
    
  default:
    break;
  }
  return status;
}


/**
* @brief  USBH_Get_HID_ReportDescriptor
*         Issue report Descriptor command to the device. Once the response 
*         received, parse the report descriptor and update the status.
* @param  pdev   : Selected device
* @param  Length : HID Report Descriptor Length
* @retval USBH_Status : Response for USB HID Get Report Descriptor Request
*/
/*
 @brief  USBH_Get_HID_ReportDescriptor
         ���豸���ͱ������������һ���յ���Ӧ����������������������״̬��
 @param  pdev   : ѡ�����豸
 @param  Length : HID��������������
 @retval USBH_Status : USB HID��ȡ�����������������Ӧ
*/

static USBH_Status USBH_Get_HID_ReportDescriptor (USB_OTG_CORE_HANDLE *pdev,
                                                  USBH_HOST *phost,
                                                  uint16_t length)
{
  
  USBH_Status status;
  
  status = USBH_GetDescriptor(pdev,
                              phost,
                              USB_REQ_RECIPIENT_INTERFACE
                                | USB_REQ_TYPE_STANDARD,                                  
                                USB_DESC_HID_REPORT, 
                                pdev->host.Rx_Buffer,
                                length);
  
  /* HID report descriptor is available in pdev->host.Rx_Buffer.
  In case of USB Boot Mode devices for In report handling ,
  HID report descriptor parsing is not required.
  In case, for supporting Non-Boot Protocol devices and output reports,
  user may parse the report descriptor*/
  /* HID��������������pdev->host.Rx_Buffer���ҵ���
  ����USB����ģʽ�豸�����뱨�洦������Ҫ����HID������������
  ����֧�ַ�����Э���豸���������������
  �û�������Ҫ�������������� */

  
  return status;
}

/**
* @brief  USBH_Get_HID_Descriptor
*         Issue HID Descriptor command to the device. Once the response 
*         received, parse the report descriptor and update the status.
* @param  pdev   : Selected device
* @param  Length : HID Descriptor Length
* @retval USBH_Status : Response for USB HID Get Report Descriptor Request
*/
/*
 @brief  USBH_Get_HID_Descriptor
       ���豸����HID���������һ���յ���Ӧ��
       ��������������������״̬��
 @param  pdev   : ѡ�����豸
 @param  Length : HID����������
 @retval USBH_Status : USB HID��ȡ�����������������Ӧ
*/

static USBH_Status USBH_Get_HID_Descriptor (USB_OTG_CORE_HANDLE *pdev,
                                            USBH_HOST *phost,
                                            uint16_t length)
{
  
  USBH_Status status;
  
  status = USBH_GetDescriptor(pdev, 
                              phost,
                              USB_REQ_RECIPIENT_INTERFACE
                                | USB_REQ_TYPE_STANDARD,                                  
                                USB_DESC_HID,
                                pdev->host.Rx_Buffer,
                                length);
 
  return status;
}

/**
* @brief  USBH_Set_Idle
*         Set Idle State. 
* @param  pdev: Selected device
* @param  duration: Duration for HID Idle request
* @param  reportID : Targeted report ID for Set Idle request
* @retval USBH_Status : Response for USB Set Idle request
*/
/*
 @brief  USBH_Set_Idle
         ���ÿ���״̬�� 
 @param  pdev: ѡ�����豸
 @param  duration: HID��������ĳ���ʱ��
 @param  reportID : ���ÿ��������Ŀ�걨��ID
 @retval USBH_Status : USB���ÿ����������Ӧ
*/

static USBH_Status USBH_Set_Idle (USB_OTG_CORE_HANDLE *pdev,
                                  USBH_HOST *phost,
                                  uint8_t duration,
                                  uint8_t reportId)
{
  
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE |\
    USB_REQ_TYPE_CLASS;
  
  
  phost->Control.setup.b.bRequest = USB_HID_SET_IDLE;
  phost->Control.setup.b.wValue.w = (duration << 8 ) | reportId;
  
  phost->Control.setup.b.wIndex.w = 0;
  phost->Control.setup.b.wLength.w = 0;
  
  return USBH_CtlReq(pdev, phost, 0 , 0 );
}


/**
* @brief  USBH_Set_Report
*         Issues Set Report 
* @param  pdev: Selected device
* @param  reportType  : Report type to be sent
* @param  reportID    : Targeted report ID for Set Report request
* @param  reportLen   : Length of data report to be send
* @param  reportBuff  : Report Buffer
* @retval USBH_Status : Response for USB Set Idle request
*/
/*
@brief  USBH_Set_Report
       �������ñ���
 @param  pdev: ѡ�����豸
 @param  reportType  : Ҫ���͵ı�������
 @param  reportID    : ���ñ��������Ŀ�걨��ID
 @param  reportLen   : Ҫ���͵����ݱ���ĳ���
 @param  reportBuff  : ���滺����
 @retval USBH_Status : USB���ÿ����������Ӧ
*/

USBH_Status USBH_Set_Report (USB_OTG_CORE_HANDLE *pdev, 
                                 USBH_HOST *phost,
                                    uint8_t reportType,
                                    uint8_t reportId,
                                    uint8_t reportLen,
                                    uint8_t* reportBuff)
{
  
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE |\
    USB_REQ_TYPE_CLASS;
  
  
  phost->Control.setup.b.bRequest = USB_HID_SET_REPORT;
  phost->Control.setup.b.wValue.w = (reportType << 8 ) | reportId;
  
  phost->Control.setup.b.wIndex.w = 0;
  phost->Control.setup.b.wLength.w = reportLen;
  
  return USBH_CtlReq(pdev, phost, reportBuff , reportLen );
}


/**
* @brief  USBH_Set_Protocol
*         Set protocol State.
* @param  pdev: Selected device
* @param  protocol : Set Protocol for HID : boot/report protocol
* @retval USBH_Status : Response for USB Set Protocol request
*/
/*
@brief  USBH_Set_Protocol
         ����Э��״̬��
 @param  pdev: ѡ�����豸
 @param  protocol : HID������Э�飺����Э��/����Э��
 @retval USBH_Status : USB����Э���������Ӧ
*/

static USBH_Status USBH_Set_Protocol(USB_OTG_CORE_HANDLE *pdev,
                                     USBH_HOST *phost,
                                     uint8_t protocol)
{
  
  
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_RECIPIENT_INTERFACE |\
    USB_REQ_TYPE_CLASS;
  
  
  phost->Control.setup.b.bRequest = USB_HID_SET_PROTOCOL;
  
  if(protocol != 0)
  {
    /* Boot Protocol */
    phost->Control.setup.b.wValue.w = 0;
  }
  else
  {
    /*Report Protocol*/
    phost->Control.setup.b.wValue.w = 1;
  }
  
  phost->Control.setup.b.wIndex.w = 0;
  phost->Control.setup.b.wLength.w = 0;
  
  return USBH_CtlReq(pdev, phost, 0 , 0 );
  
}

/**
* @brief  USBH_ParseHIDDesc 
*         This function Parse the HID descriptor
* @param  buf: Buffer where the source descriptor is available
* @retval None
*/
/*
@brief  USBH_ParseHIDDesc 
         �˺�������HID������
 @param  buf: ����Դ�������Ļ�����
 @retval None
*/

static void  USBH_ParseHIDDesc (USBH_HIDDesc_TypeDef *desc, uint8_t *buf)
{
  
  desc->bLength                  = *(uint8_t  *) (buf + 0);
  desc->bDescriptorType          = *(uint8_t  *) (buf + 1);
  desc->bcdHID                   =  LE16  (buf + 2);
  desc->bCountryCode             = *(uint8_t  *) (buf + 4);
  desc->bNumDescriptors          = *(uint8_t  *) (buf + 5);
  desc->bReportDescriptorType    = *(uint8_t  *) (buf + 6);
  desc->wItemLength              =  LE16  (buf + 7);
  
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


/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
