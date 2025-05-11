#include "imu_task.h"

//imu���ݽṹ��
IMU_DATA_t imu;

void MPU6050_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        //This task runs at 100Hz
        //��������100Hz��Ƶ������
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_TASK_RATE));
        //Read the gyroscope zero before starting
        //����ǰ����ȡ���������
        if(SysVal.Time_count<CONTROL_DELAY)
        {	
            ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
            ImuData_copy(&imu.Deviation_accel,&imu.accel);
        }
        //Get acceleration sensor data
        MPU6050_Get_Accelscope();

        //Get gyroscope data
        MPU6050_Get_Gyroscope(); //�õ�����������

    }
}


void ICM20948_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        //This task runs at 100Hz
        //��������100Hz��Ƶ������
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_TASK_RATE));

        //Read the gyroscope zero before starting
        //����ǰ����ȡ���������
        if(SysVal.Time_count<CONTROL_DELAY)
        {
            ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
            ImuData_copy(&imu.Deviation_accel,&imu.accel);
        }

        //Get acceleration sensor data
        ICM20948_Get_Accel(); //�õ����ٶȴ���������

        //Get gyroscope data
        ICM20948_Get_Gyroscope(); //�õ�����������

#if 0 // δʹ�ô���������,�������ɼ�
        static u8 mag_count=0;
        //Get magnetometer data
        mag_count++;
        if(mag_count>=13)	 //����������������Ϊ8hz�����ÿ�����������13�βŲ���һ�δ�������Ϣ
        {
            invMSMagRead(&magnet[0], &magnet[1], &magnet[2]); //�õ�����������
            mag_count=0;
        }
#endif


    }
}


/**************************************************************************
�������ܣ�����imu����ֵ
��ڲ�������Ҫ��ֵ�Ľṹ������������ƵĽṹ�����
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void ImuData_copy(IMU_BASE_t* req_val,const IMU_BASE_t* copied_val)
{
	req_val->x = copied_val -> x;
	req_val->y = copied_val -> y;
	req_val->z = copied_val -> z;
}


