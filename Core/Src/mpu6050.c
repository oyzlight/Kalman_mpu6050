#include "mpu6050.h"
#include "i2c.h"
#include "stdio.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
static int16_t Mpu6050Addr = 0xD0;
static uint32_t last_tick =0;
const uint16_t i2c_timeout = 100;
float dt=0.0f;
#define USE_I2C 1
#if USE_I2C==1    
#define MPU6050_I2C hi2c1
#elif USE_I2C==2   
#define MPU6050_I2C hi2c2
#endif   

#define RAD_TO_DEG 57.295779513082320876798154814105

int8_t Sensor_I2C_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Read(&MPU6050_I2C,DevAddr,MemAddr,1,oData,DataLen,10);
}

int8_t Sensor_I2C_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Write(&MPU6050_I2C,DevAddr,MemAddr,1,iData,DataLen,10);
}

int16_t Sensor_I2C_Serch(void)
{
	for(uint8_t i = 1; i < 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&MPU6050_I2C, i, 1, 10) == HAL_OK)
		{
			Mpu6050Addr = i>>1;
			return i>>1;
		}
	}
	return -1; // 搜索失败
}

int8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;
 
    // check device ID WHO_AM_I
 
    HAL_I2C_Mem_Read(I2Cx, Mpu6050Addr, WHO_AM_I, 1, &check, 1, i2c_timeout);
 
    if (check == 112) // 0x70 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, Mpu6050Addr, PWR_MGMT_1, 1, &Data, 1, i2c_timeout);
 
        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, Mpu6050Addr, SMPLRT_DIV, 1, &Data, 1, i2c_timeout);
 
        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, Mpu6050Addr, ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
 
        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, Mpu6050Addr, GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}
void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Read_Buf[6];
	
	// 寄存器依次是加速度X高 - 加速度X低 - 加速度Y高位 - 加速度Y低位 - 加速度Z高位 - 加速度度Z低位
	HAL_I2C_Mem_Read(I2Cx, Mpu6050Addr, ACCEL_XOUT_H, 1, Read_Buf, 6, i2c_timeout);
	
	Mpu6050.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050.Accel_X = Mpu6050.Accel_X / 16384.0f;
	Mpu6050.Accel_Y = Mpu6050.Accel_Y / 16384.0f;
	Mpu6050.Accel_Z = Mpu6050.Accel_Z / 16384.0f;
	
}
void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Read_Buf[6];
	
	// 寄存器依次是角度X高 - 角度X低 - 角度Y高位 - 角度Y低位 - 角度Z高位 - 角度Z低位
	HAL_I2C_Mem_Read(I2Cx, Mpu6050Addr, GYRO_XOUT_H, 1, Read_Buf, 6, i2c_timeout);
	
	Mpu6050.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050.Gyro_X = Mpu6050.Gyro_X / 131.0f;
	Mpu6050.Gyro_Y = Mpu6050.Gyro_Y / 131.0f;
	Mpu6050.Gyro_Z = Mpu6050.Gyro_Z / 131.0f;
	
}
void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx)
{
    uint8_t Read_Buf[2];
	HAL_I2C_Mem_Read(I2Cx, Mpu6050Addr, TEMP_OUT_H, 1, Read_Buf, 2, i2c_timeout);
	
	Mpu6050.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	
	Mpu6050.Temp = 36.53f + (Mpu6050.Temp / 340.0f);
}
void MPU6050_Calibrate_Gyro_offset(void)
{
    float gx = 0.0f;
    float gy = 0.0f;
   
    float ax = 0.0f;
    float ay = 0.0f;
  
    for(uint32_t i=0;i<2000;i++)
    {
       MPU6050_Read_Gyro(&hi2c1);
       gx += Mpu6050.Gyro_X;
       gy += Mpu6050.Gyro_Y;
     
       MPU6050_Read_Accel(&hi2c1);
       ax += Mpu6050.Accel_X;
       ay += Mpu6050.Accel_Y;
      

        if(i % 100 == 0)  // 每100次打印一次
        {
        printf("Progress: %ld/500\r\n", i);
        }
    }
    Mpu6050.Gyro_X_offset = gx / 2000.0f;
    Mpu6050.Gyro_Y_offset = gy / 2000.0f;
  
    Mpu6050.Accel_X_offset = ax / 2000.0f;
    Mpu6050.Accel_Y_offset = ay / 2000.0f;
  
    // HAL_Delay(100);
}

void MPU6050_Read_All(void)
{
    uint8_t Read_Buf[14];
    uint32_t tick = HAL_GetTick();
    dt = (tick -last_tick)/1000.0f;
    last_tick = tick;
    
    if(HAL_OK != Sensor_I2C_Read(Mpu6050Addr, ACCEL_XOUT_H, Read_Buf, 14))
    {
        printf("MPU6050_Read_All Error!\r\n");
        return;
    }
    
    Mpu6050.Accel_X_Raw = (int16_t)((uint16_t)Read_Buf[0] << 8 | Read_Buf[1]); /* 加速度传感器X轴 */
    Mpu6050.Accel_Y_Raw = (int16_t)((uint16_t)Read_Buf[2] << 8 | Read_Buf[3]); /* 加速度传感器Y轴 */
    Mpu6050.Accel_Z_Raw = (int16_t)((uint16_t)Read_Buf[4] << 8 | Read_Buf[5]); /* 加速度传感器Z轴 */
    
    Mpu6050.Temp_Raw = (int16_t)((uint16_t)Read_Buf[6] << 8 | Read_Buf[7]);
    
    Mpu6050.Gyro_X_Raw = (int16_t)((uint16_t)Read_Buf[8] << 8 | Read_Buf[9]);/*角速度传感器X轴*/
    Mpu6050.Gyro_Y_Raw = (int16_t)((uint16_t)Read_Buf[10] << 8 | Read_Buf[11]);/*角速度传感器Y轴*/
    Mpu6050.Gyro_Z_Raw = (int16_t)((uint16_t)Read_Buf[12] << 8 | Read_Buf[13]);/*角速度传感器Z轴*/
    
    Mpu6050.Accel_X = Mpu6050.Accel_X_Raw / 16384.0f;
    Mpu6050.Accel_Y = Mpu6050.Accel_Y_Raw / 16384.0f;   
    Mpu6050.Accel_Z = Mpu6050.Accel_Z_Raw / 16384.0f;
    
    Mpu6050.Gyro_X = Mpu6050.Gyro_X_Raw / 131.0f;
    Mpu6050.Gyro_Y = Mpu6050.Gyro_Y_Raw / 131.0f;
    Mpu6050.Gyro_Z = Mpu6050.Gyro_Z_Raw / 131.0f;
    
    Mpu6050.Temp = 36.53f + (Mpu6050.Temp_Raw / 340.0f);

}
void Kalman_Init(void)
{
    Kalman.Q[0][0]=0.0025f; /*过程噪声协方差矩阵*/
    Kalman.Q[0][1]=0.0f;
    Kalman.Q[1][0]=0.0f;
    Kalman.Q[1][1]=0.0025f;

    Kalman.P[0][0]=1.0f;   /*误差协方差矩阵*/
    Kalman.P[0][1]=0.0f;
    Kalman.P[1][0]=0.0f;
    Kalman.P[1][1]=1.0f;

    Kalman.R[0][0]=0.3f;
    Kalman.R[0][1]=0.0f;
    Kalman.R[1][0]=0.0f;
    Kalman.R[1][1]=0.3f;  /*测量噪声协方差矩阵*/

    Kalman.K_p=0.0f;
    Kalman.K_r=0.0f;

}
void Kalman_Update(void)
{
   Mpu6050.V_roll= (Mpu6050.Gyro_X-Mpu6050.Gyro_X_offset)+(sin(Kalman.K_p)*sin(Kalman.K_r)/cos(Kalman.K_p))*
   (Mpu6050.Gyro_Y-Mpu6050.Gyro_Y_offset)+(sin(Kalman.K_p)*cos(Kalman.K_r)/cos(Kalman.K_p))*(Mpu6050.Gyro_Z-Mpu6050.Gyro_Z_offset);

   Mpu6050.V_pitch=(cos(Kalman.K_r))*(Mpu6050.Gyro_Y-Mpu6050.Gyro_Y_offset)+
   (-sin(Kalman.K_r))*(Mpu6050.Gyro_Z-Mpu6050.Gyro_Z_offset);

   Kalman.Gyro_roll=Kalman.K_r+dt*Mpu6050.V_roll;
   Kalman.Gyro_pitch=Kalman.K_p+dt*Mpu6050.V_pitch;

    /*计算先验误差p协方差矩阵*/
    Kalman.P[0][0]=Kalman.P[0][0]+Kalman.Q[0][0];
    Kalman.P[0][1]=Kalman.P[0][1]+Kalman.Q[0][1];
    Kalman.P[1][0]=Kalman.P[1][0]+Kalman.Q[1][0];
    Kalman.P[1][1]=Kalman.P[1][1]+Kalman.Q[1][1];

    /*更新卡尔曼增益*/
    Kalman.K[0][0]=Kalman.P[0][0]/(Kalman.P[0][0]+Kalman.R[0][0]);
    Kalman.K[0][1]=Kalman.P[0][1]/(Kalman.P[0][1]+Kalman.R[0][1]);
    Kalman.K[1][0]=Kalman.P[1][0]/(Kalman.P[1][0]+Kalman.R[1][0]);
    Kalman.K[1][1]=Kalman.P[1][1]/(Kalman.P[1][1]+Kalman.R[1][1]);

    /*更新状态*/
    Kalman.Accel_roll=atan2(Mpu6050.Accel_Y-Mpu6050.Accel_Y_offset,Mpu6050.Accel_Z)*RAD_TO_DEG;
    Kalman.Accel_pitch=atan2(Mpu6050.Accel_X-Mpu6050.Accel_X_offset,sqrt((Mpu6050.Accel_Y-Mpu6050.Accel_Y_offset)*(Mpu6050.Accel_Y-Mpu6050.Accel_Y_offset)
   +(Mpu6050.Accel_Z-Mpu6050.Accel_Z_offset)*(Mpu6050.Accel_Z-Mpu6050.Accel_Z_offset)))*RAD_TO_DEG;


    /*最优估计状态*/
    Kalman.K_r=Kalman.Gyro_roll+Kalman.K[0][0]*(Kalman.Accel_roll-Kalman.Gyro_roll);
    Kalman.K_p=Kalman.Gyro_pitch+Kalman.K[1][1]*(Kalman.Accel_pitch-Kalman.Gyro_pitch);

    /*更新误差p*/
    Kalman.P[0][0]=(1-Kalman.K[0][0])*Kalman.P[0][0];
    Kalman.P[0][1]=(1-Kalman.K[0][1])*Kalman.P[0][1];
    Kalman.P[1][0]=(1-Kalman.K[1][0])*Kalman.P[1][0];
    Kalman.P[1][1]=(1-Kalman.K[1][1])*Kalman.P[1][1];

    printf("r_p:%f,%f\n",Kalman.K_r,Kalman.K_p);
   



}