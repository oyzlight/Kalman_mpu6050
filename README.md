# Kalman_mpu6050

## MUP6050卡尔曼预测+QMC5883P磁力计
后续补充加磁力计
### 加速度计：

### 基础算法

#### 横滚角roll

滚角 roll 指：重力加速度与“X-Z 平面”的夹角。上图中重力加速度在 X-Z 平面的投影正好与 z 轴重合，这是一种特殊情况。应用勾股定理推广到一般的情况是：`重力加速度在 X-Z 平面的投影长度 = sqrt(Ax^2 + Az^2)`。因此，我们能够得到 Ф 在三维空间内的通用公式：

```c
Ф = arctan(Ay / sqrt(Ax^2 + Az^2))
```

#### 俯仰角pitch

```
θ = arctan(-Ax / sqrt(Ay^2 + Az^2))
```

提醒：θ 表示重力加速度与“Y-Z 平面”的夹角。

### 陀螺仪：

在姿态解算上我们关心的是`向量角度`而非向量模长，为了能够顺利的使用`反三角函数`做姿态解算，我们必须首先对加速度计数据进行**`单位化`**，保证反三角函数有`实数解`。假设上图的`红色箭头(直线的那个)`是通过对`单位向量[0;0;1]`施加三维旋转得到的。为了便于表示，我们做如下约定：

```
% cr:cos(roll)  sr:sin(roll)
% cp:cos(pitch) sp:sin(pitch)
% cy:cos(yaw)   sy:sin(yaw)
from_euler_xyz = 
[ cp*cy, cy*sp*sr - cr*sy, sr*sy + cr*cy*sp]
[ cp*sy, cr*cy + sp*sr*sy, cr*sp*sy - cy*sr]
[   -sp,            cp*sr,            cp*cr]
```

在这样的约定下，我们有：

```
[加速度计单位化后的读数] = from_euler_xyz * [0;0;1];
```

记`加速度计单位化后的读数`为`[Ax;Ay;Az]`，将上面方程的右侧展开，可得：

```
 Ax = sr*sy + cr*cy*sp 
 Ay = cr*sp*sy - cy*sr 
 Az =            cp*cr
```

这个旋转矩阵的作用下，单位向量在 Z 轴上的投影只与`横滚角 roll`和`俯仰角 pitch`有关

在 from_euler_xyz 旋转矩阵的作用下，各个偏航角不同的姿态对应的横滚角 roll 和俯仰角 pitch 都是跟**`偏航角为 0 的姿态`**是一样的。因为用 IMU 无法估算偏航角 Yaw，不妨随意标定一个姿态是 Yaw 为 0 的姿态，这样就有 `sy = 0`、`cy = 1`，这样 `Ax、Ay 和 Az` 就是：

```c
 Ax = cr*sp
 Ay = -sr 
 Az = cp*cr
```



```c
r = -asin(Ay)
p = atan2(Ax,Az)
```

基本算法的atan只能返回-90，90，而atan2-Π，Π



### Kalman滤波器与投影

**一、确定目标量**

姿态解算我们关注的是：`横滚角`、`横滚角速度`、`俯仰角`和`俯仰角速度`。把这些目标量装到 `state_estimate` 列向量里，如下：

```
state_estimate = [横滚角; 横滚角速度; 俯仰角; 俯仰角速度];
```

为了便于表示，我们把文字改成英文：

```
state_estimate = [phi; phi_rate; theta; theta_rate];
```

**二、确定状态转移矩阵**

```
A = 
[ 1, dt, 0,  0]
[ 0,  1, 0,  0]
[ 0,  0, 1, dt]
[ 0,  0, 0,  1]
```

为什么 `A` 长这样的？我们看下 MATLAB 怎么说：

```
>> A*state_estimate ans =      
phi + dt*phi_rate              
phi_rate theta + dt*theta_rate            
theta_rate
```

这样容易看出：结果仍然符合我们第一步确定的目标量的排列方式。

**三、确定要不要有 `B\*u(t)`**

在姿态解算中，我们的目标量是“横滚角”、“俯仰角”、“横滚角速度”和“俯仰角速度”，这些都可以通过 MPU6050 观测到，没有影响目标量的额外因素。因此，在本篇的姿态解算中`不需要 B*u(t)`。

**四、系统噪声 Q**

系统噪声 Q 是留出的手动调节参数，看下 Sugar 在 MATLAB 里是怎么把这个调节接口留出来的：

```c
var_acc_init   = 1e-2;var_gyro_init  = 6.75e0;
var_acc        = [var_acc_init, var_acc_init]';
var_gyro       = [var_gyro_init, var_gyro_init]';
Q = eye(4);
Q(1,1) = Q(1,1) * var_acc(1);
Q(2,2) = Q(2,2) * var_gyro(1);
Q(3,3) = Q(3,3) * var_acc(2);
Q(4,4) = Q(4,4) * var_gyro(2);
```

**五、观测噪声 R**

观测噪声要随采样计算，Sugar 在 MATLAB 里取的是`每一时刻的前 10 个采样值的方差`给观测噪声 R，MATLAB 的程序是这样的：



```c
cnt = 10;if i>cnt    
var_acc(:,i)  = [var(phi_acc(size(phi_acc,1)-cnt:end)), var(theta_acc(size(theta_acc,1)-cnt:end))]';  
var_gyro(:,i) = [var(phi_rate_gyro(size(phi_rate_gyro,1)-cnt:end)), var(theta_rate_gyro(size(theta_rate_gyro,1)-cnt:end))]';else    var_acc(:,i)  = [var(phi_acc), var(theta_acc)]';   
var_gyro(:,i) = [var(phi_rate_gyro), var(theta_rate_gyro)]';end
R = eye(4);
R(1,1) = R(1,1) * var_acc(1,i);
R(2,2) = R(2,2) * var_gyro(1,i);
R(3,3) = R(3,3) * var_acc(2,i);
R(4,4) = R(4,4) * var_gyro(2,i);
```

**六、应用线性 Kalam 的 5 个公式开始递推**

这一步没太多要说的，直接用上面介绍过的 5 个公式就行了，如下：

```c
state_estimate = A * state_estimate;
P = A * P * A' + Q;
K = P * C' * inv(R + C * P * C');
state_estimate 
= state_estimate + K * (measurement - C * state_estimate);
P = (eye(4) - K * C) * P;
```



![image-20260412173602945](C:\Users\slope\AppData\Roaming\Typora\typora-user-images\image-20260412173602945.png)



```c
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
```



