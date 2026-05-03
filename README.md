# Kalman_mpu6050

## MUP6050卡尔曼预测+QMC5883P磁力计
**注**：本算法基于陀螺仪积分 + 加速度计观测实现 2D Kalman 滤波，分别输出横滚角 roll 和俯仰角 pitch。地磁计 yaw 单独处理，详见下方地磁计章节。
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

---

## QMC5883P 地磁计与 Yaw 角解算

### 传感器连接

| 信号 | STM32F407 引脚 | 说明 |
|------|---------------|------|
| SCL | PB10 (I2C2_SCL) | I2C 时钟线，100kHz |
| SDA | PB11 (I2C2_SDA) | I2C 数据线 |
| DRDY | PB3 | 数据就绪中断（可选） |
| VCC | 3.3V | 电源 |
| GND | GND | 地 |

I2C2 地址为 `0x1A`（默认 7 位地址），与 MPU6050 共用 I2C 总线但不冲突（MPU6050 使用 I2C1）。

### 磁场数据读取流程

1. `QMC5883P_Init(&hi2c2)` — 配置控制寄存器 1，设置为连续模式、200Hz 输出速率、±2 Gauss 量程
2. 主循环中调用 `QMC5883P_Ready(&hi2c2)` 判断 DRDY 引脚或状态寄存器
3. 调用 `QMC5883P_ReadData(&hi2c2, &data)` 读取三轴原始值（16 位有符号）
4. 减去硬磁偏移量（通过 `校准.html` 页面椭球拟合标定得到）
5. 使用 `atan2f()` 计算方位角

### 方位角计算公式

```
heading = atan2f(-calibrated_x, -calibrated_y)
```

符号取负是因为 QMC5883P 的坐标轴方向与地理坐标系约定相反：
- X 指向传感器前方，但磁场 X 分量在正北时需要映射为正角度
- Y 指向传感器右侧

### 硬磁校准

打开 `校准.html`（浏览器端椭球拟合工具），将 QMC5883P 在三维空间中各方向缓慢旋转，收集 ~200 个采样点后点击"计算"。工具输出 `QMC5883P_OFFSET_X` 和 `QMC5883P_OFFSET_Y`，写入 `main.c` 中的宏定义即可。

---

## 低通滤波与 sin/cos 分解方法

### 问题：直接对角度做低通滤波会出错

角度是一个**循环量**（0° 和 360° 是同一个方向），直接使用一阶低通滤波：

```c
filtered = prev + alpha * (raw - prev);
```

当 yaw 从 359° 变成 1° 时（实际只转了 2°），滤波器却认为变化量是 `1 - 359 = -358°`，导致输出剧烈抖动。

### 解决方案：sin/cos 分解滤波

将角度**分解为单位圆上的点** `(cos θ, sin θ)`，对两个分量分别做低通滤波，再通过 `atan2` 还原角度。

```
sin_h = sinf(heading);
cos_h = cosf(heading);
filter_yaw_sin = low_pass_filter(&sin_h, &filter_yaw_sin, 0.2f);
filter_yaw_cos = low_pass_filter(&cos_h, &filter_yaw_cos, 0.2f);
yaw = atan2f(filter_yaw_sin, filter_yaw_cos);
```

### 为什么有效

- `cos θ` 和 `sin θ` 是连续函数，θ 从 359°→0° 时分量几乎不变
- 两个分量独立低通滤波后，在单位圆上的点位置平滑移动
- `atan2` 复原的角度自然绕过 0°/360° 跳变点

### 低通滤波函数实现

```c
float low_pass_filter(float *input, float *last, float alpha)
{
    *last = *last + alpha * (*input - *last);
    return *last;
}
```

- **alpha**：滤波系数，范围 `(0, 1]`
  - `alpha = 1.0`：无滤波，完全跟踪原始值
  - `alpha = 0.1`：强滤波，响应慢但平滑
  - **推荐值 `0.2`**：兼顾响应速度与平滑度（对应 ~2.5Hz 截止频率 @200Hz 采样率）

---

## 360°/0° 附近角度跳变的解决

### 角度归一化与边界处理

即使使用 sin/cos 分解滤波，`atan2f` 返回的角度仍然是 `[-π, +π]` 弧度（即 `[-180°, +180°]`）。转换为 0~360° 后还需处理浮点数累积带来的边界溢出。

### 归一化代码

```c
yaw = yaw * 180.0f / M_PI;       // 弧度转角度

// 归一化到 [0, 360)
if (yaw < 0.0f)
    yaw += 360.0f;
else if (yaw >= 360.0f)
    yaw -= 360.0f;
```

### 边界跳变的三种场景与处理

| 场景 | 原始值 | 处理方式 | 输出 |
|------|--------|---------|------|
| 偏北略偏西 | -1° (即 359°) | `yaw < 0` → `+360` | 359° |
| 偏北略偏东 | 0.5° | 不变 | 0.5° |
| 浮点累积溢出 | 360.001° | `yaw >= 360` → `-360` | 0.001° |

### 为什么不用 `fmod`

```c
// 不推荐：fmod 在 360.0 附近有精度损失
yaw = fmodf(yaw, 360.0f);

// 不推荐：对负数取模结果与期望不符
// fmodf(-1.0f, 360.0f) = -1.0f  ← 期望返回 359.0f
```

手写 if 分支避免了浮点取模运算的精度和语义问题，在 ARM Cortex-M4F 上也比 `fmodf` 更快（`fmodf` 需要库函数调用）。

### 实际效果

配合 sin/cos 低通滤波后，即使传感器输出快速跨越 0°/360° 边界，滤波后的角度过渡平滑，不会出现 ~360° 的尖峰跳变。在日志中表现为角度在 `359.8° → 0.2° → 1.5°` 的顺序变化，而非 `359.8° → 359.5° → 1.5°` 的异常跳回。



