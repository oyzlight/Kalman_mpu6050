/*
 * @Author: laptoy laptoy458@163.com
 * @Date: 2026-04-15 19:59:40
 * @LastEditors: laptoy laptoy458@163.com
 * @LastEditTime: 2026-04-16 16:05:51
 * @FilePath: \Kalman_mpu6050\Core\Inc\mpu6050.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

#define SMPLRT_DIV   0x19  // 采样率分频，典型值：0x07(125Hz) */
#define CONFIG       0x1A  // 低通滤波频率，典型值：0x06(5Hz) */
#define GYRO_CONFIG  0x1B  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) */
#define ACCEL_CONFIG 0x1C  // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) */

#define ACCEL_XOUT_H 0x3B  // 存储最近的X轴、Y轴、Z轴加速度感应器的测量值 */
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H   0x41  // 存储的最近温度传感器的测量值 */
#define TEMP_OUT_L   0x42

#define GYRO_XOUT_H  0x43  // 存储最近的X轴、Y轴、Z轴陀螺仪感应器的测量值 */
#define GYRO_XOUT_L  0x44 
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define PWR_MGMT_1   0x6B   // 电源管理，典型值：0x00(正常启用) */
#define WHO_AM_I     0x75 	// IIC地址寄存器(默认数值0x68，只读) */

// HAL库的读写只需要使用7位地址
#define MPU6050_ADDR_AD0_LOW 0x68	// AD0低电平时7位地址为0X68 iic写时许发送0XD0
#define MPU6050_ADDR_AD0_HIGH 0x69	

typedef struct{
	// 角速度
	float Accel_X;
	float Accel_Y;
	float Accel_Z;
	// 角度
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;
	// 温度
	float Temp;

	float Accel_X_Raw;
	float Accel_Y_Raw;
	float Accel_Z_Raw;

	float Gyro_X_Raw;
	float Gyro_Y_Raw;
	float Gyro_Z_Raw;
	float Temp_Raw;

	float V_roll;
	float V_pitch;

	float Gyro_X_offset;
	float Gyro_Y_offset;
	float Gyro_Z_offset;
	float Accel_X_offset;
	float Accel_Y_offset;
	float Accel_Z_offset;
}Mpu6050_t;

typedef struct{
	float K_p;
	float K_r;

	float Gyro_roll;
	float Gyro_pitch;
	float Accel_roll;
	float Accel_pitch;

	float Q[2][2];
	float R[2][2];
	float P[2][2];
	float K[2][2];
}Kalman_t;

extern Mpu6050_t Mpu6050;
extern Kalman_t Kalman;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

int16_t Sensor_I2C2_Serch(void);
int8_t MPU6050_Init();

int8_t Sensor_I2C_ReadOneByte(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData);
int8_t Sensor_I2C_WriteOneByte(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx);
void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx);
void MPU6050_Read_All(void);
void MPU6050_Calibrate_Gyro_offset(void);
void Kalman_Update(void);
void Kalman_Init(void);

#endif



