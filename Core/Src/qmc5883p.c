#include "QMC5883P.h"
#include <stdint.h>
#include "i2c.h"

// 使用 HAL_I2C_Mem_Write 替代原有的数组发送方式
uint8_t QMC5883P_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hi2c, QMC5883P_ADDR<<1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

// 修复：改用 Mem_Read 并精准提取 DRDY 位
uint8_t QMC5883P_Ready(I2C_HandleTypeDef *hi2c)
{
    uint8_t status = 0;
    // 使用 Mem_Read 产生标准的 Repeated Start 信号
    if(HAL_I2C_Mem_Read(hi2c, QMC5883P_ADDR<<1|0x01, QMC5883P_REG_STATUS, I2C_MEMADD_SIZE_8BIT, &status, 1, 100) == HAL_OK)    
    {
        // 仅当 DRDY (Data Ready, 也就是 bit 0) 为 1 时返回真
        return (status & 0x01);
    }
    return 0; // 通信失败或数据未就绪
}

// 修复：改用 Mem_Read 连续读取 6 个字节
uint8_t QMC5883P_ReadData(I2C_HandleTypeDef *hi2c, QMC5883P_Data_t *data) {
    uint8_t buffer[6];

    if (HAL_I2C_Mem_Read(hi2c, QMC5883P_ADDR<<1|0x01, QMC5883P_REG_DATA, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100) != HAL_OK) {
        return 1; // 读取失败
    }

    // 拼接 16 位数据 (低位在前，高位在后)
    data->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    data->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    data->z = (int16_t)(buffer[4] | (buffer[5] << 8));

    return 0;
}

uint8_t QMC5883P_Init(I2C_HandleTypeDef *hi2c) {
    HAL_Delay(10);
   // 2. 配置核心控制寄存器 (QMC5883P 的控制寄存器在 0x0A)
    // 写入 0xCF (二进制 11 00 11 11) 的含义：
    // [7:6] OSR  = 11 (高过采样率，降低噪声)
    // [5:4] RNG  = 00 (量程 2 Gauss，适用于常规地磁测量)
    // [3:2] ODR  = 11 (输出数据速率 200Hz，刷新很快)
    // [1:0] MODE = 11 (连续测量模式，芯片会自动一直转换数据)
    QMC5883P_WriteReg(hi2c, 0x0A, 0xCF);
    
    // (可选) 如果你发现数据有时卡死，可以加上软件复位和置位：
    // QMC5883P_WriteReg(hi2c, 0x0B, 0x80); // 写入 0x0B 软复位
    return 0;
}
