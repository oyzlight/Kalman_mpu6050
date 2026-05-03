#ifndef __QMC5883P_H__
#define __QMC5883P_H__

#include <stdint.h>
#include "i2c.h"

#define QMC5883P_ADDR         0x2C

#define QMC5883P_ADDR_WRITE 0x58  // 7位地址 0x2C 左移1位后为 0x58
#define QMC5883P_REG_DATA   0x01  // 数据起始寄存器
#define QMC5883P_REG_STATUS 0x09  // 状态寄存器
#define QMC5883P_REG_CTRL   0x0A  // 控制寄存器
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temp;
}QMC5883P_Data_t;

uint8_t QMC5883P_Init(I2C_HandleTypeDef *hi2c);
uint8_t QMC5883P_ReadData(I2C_HandleTypeDef *hi2c, QMC5883P_Data_t *data);
uint8_t QMC5883P_Ready(I2C_HandleTypeDef *hi2c);
#endif /* __QMC5883P_H__ */