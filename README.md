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




