# 从 IMU 姿态估计到三轴云台控制

三轴云台怎样根据 IMU(Inertial Measurement Unit)的数据估计空间姿态，再把姿态误差变成电机力矩？

为了让公式的方向保持一致，全文采用以下约定：

- $\lbrace n\rbrace$ 表示导航坐标系，是近似静止的外部参考系。
- $\lbrace b\rbrace$ 表示机体坐标系，固定在云台载荷上。
- $R_b^n$ 把向量的机体系分量转换为导航系分量。
- $q_b^n$ 表示与 $R_b^n$ 相同的主动旋转，采用标量在前的 Hamilton 四元数。
- 正旋转方向服从右手定则。

不同资料可能选择 ENU（East-North-Up，东-北-天）或 NED（North-East-Down，北-东-地）导航系，也可能采用被动旋转、左乘更新或标量在后的四元数。公式符号随约定改变，但几何关系不变。阅读或实现时应先确认坐标系、旋转方向、乘法顺序和四元数分量顺序。

## 第一篇：云台控制需要解决什么问题

### 1. 电机输出的是力矩，控制目标却是姿态

先看一个只能绕固定轴转动的单轴云台。设转动惯量为 $J$，电机力矩为 $\tau_m$，摩擦力矩为 $\tau_f$，外部扰动力矩为 $\tau_d$，转轴角速度为 $\omega$。其动力学近似为：

$$
J\dot\omega=\tau_m-\tau_f-\tau_d
$$

角速度再经过积分得到角度：

$$
\dot\theta=\omega
$$

$$
\theta(t)=\theta(0)+\int_0^t\omega(\lambda)\,d\lambda
$$

物理链路可以写成：

```text
电机力矩
  -> 角加速度
  -> 角速度
  -> 角度
```

假设期望角度是 $\theta_d$，反馈控制器使用误差：

$$
e_\theta=\theta_d-\theta
$$

仅用角度误差直接控制电机时，快速性和阻尼往往难以同时兼顾。常见做法是串级控制：角度外环把角度误差变成目标角速度，角速度内环再把速度误差变成电机力矩或电流命令。

### 2. 三轴系统多了旋转顺序和轴耦合

三维旋转不可交换：

$$
R_x(\alpha)R_y(\beta)\ne R_y(\beta)R_x(\alpha)
$$

先绕 X 轴转动，再绕 Y 轴转动，与调换顺序后的最终姿态通常不同。三轴云台的机械轴还是嵌套的：内层框架转动后，外层或中层转轴在载荷坐标系中的方向也会改变。

因此，完整控制过程至少要回答四个问题：

- IMU 给出的三个分量沿什么轴？
- 当前机体相对于外部参考系处于什么姿态？
- 期望姿态与当前姿态之间差多少？
- 需要沿每个电机轴产生多大的力矩？

这些问题分别对应坐标变换、姿态估计、旋转误差和闭环控制。

### 3. IMU 直接测到的是比力和角速度

IMU 通常包含三轴陀螺仪和三轴加速度计。

陀螺仪测量机体相对于惯性参考系的角速度，并用传感器自身坐标轴给出分量。若传感器轴已经转换到机体系，可写为：

$$
\boldsymbol\omega_m^b
=\boldsymbol\omega_{b/n}^b+\mathbf b_g^b+\mathbf n_g^b
$$

$\boldsymbol\omega_{b/n}^b$ 表示机体系相对导航系的真实角速度，右上标 $b$ 表示三个分量沿机体轴表达。$\mathbf b_g^b$ 是陀螺仪零偏，$\mathbf n_g^b$ 是测量噪声。

若某一轴残余零偏为 $0.1^\circ/s$，纯积分 $60\,s$ 后会积累约 $6^\circ$ 的姿态误差：

$$
\Delta\theta(t)\approx b_gt
$$

加速度计测量比力。忽略地球自转等小量时：

$$
\mathbf f^b
=R_n^b(\mathbf a^n-\mathbf g^n)+\mathbf b_a^b+\mathbf n_a^b
$$

$\mathbf a^n$ 是传感器在导航系中的平动加速度，$\mathbf g^n$ 是重力加速度，$\mathbf b_a^b$ 和 $\mathbf n_a^b$ 分别是零偏与噪声。静止时 $\mathbf a^n=0$，因此加速度计读数方向与重力方向相反，具体正负还取决于坐标系约定。

陀螺仪短时间内响应快，但零偏会累积。加速度计能提供长期稳定的竖直参考，但平移加速、振动和冲击会污染这个参考。姿态估计的主要任务是结合两者的互补特性。

## 第二篇：用数学表示三维旋转

### 4. 坐标系改变的是分量，不是物理向量

空间中同一个物理向量 $\mathbf v$，在不同坐标系中有不同的数值分量。设传感器坐标系为 $\{s\}$，机体坐标系为 $\{b\}$，则：

$$
\mathbf v^b=R_s^b\mathbf v^s
$$

若继续转换到导航系：

$$
\mathbf v^n=R_b^n\mathbf v^b
$$

两个变换可以合并：

$$
\mathbf v^n=R_b^nR_s^b\mathbf v^s
$$

$R_b^n$ 的三列分别是机体 X、Y、Z 单位轴在导航系中的坐标。旋转不改变向量长度和向量之间的夹角，所以旋转矩阵满足：

$$
R^TR=I
$$

$$
\det R=1
$$

因此反向变换为：

$$
R_n^b=(R_b^n)^{-1}=(R_b^n)^T
$$

这里还要区分两种表述。主动旋转保持坐标系不动，让物理向量旋转；被动变换保持物理向量不动，只改变描述它的坐标系。两者使用互为转置的矩阵。全文用 $R_b^n\mathbf v^b=\mathbf v^n$ 表示分量变换，也把 $R_b^n$ 视为将机体初始方向主动旋转到导航系方向的姿态矩阵。

### 5. 从轴角到旋转矩阵

任何三维旋转都可以描述为：绕单位轴 $\mathbf u$ 旋转角度 $\theta$。设：

$$
\mathbf u=
\begin{bmatrix}
u_x\\u_y\\u_z
\end{bmatrix},
\qquad
\lVert\mathbf u\rVert=1
$$

被旋转的向量为 $\mathbf v$。先把它分解成平行于旋转轴和垂直于旋转轴的两部分：

$$
\mathbf v_\parallel
=\mathbf u(\mathbf u^T\mathbf v)
$$

$$
\mathbf v_\perp
=\mathbf v-\mathbf v_\parallel
$$

旋转不会改变平行分量。垂直分量在与 $\mathbf u$ 垂直的平面内旋转，$\mathbf u\times\mathbf v$ 给出该平面中的正交方向。旋转后的向量满足 Rodrigues 公式：

$$
\mathbf v'
=\mathbf v\cos\theta
+(\mathbf u\times\mathbf v)\sin\theta
+\mathbf u(\mathbf u^T\mathbf v)(1-\cos\theta)
$$

定义叉乘矩阵：

$$
[\mathbf u]_\times=
\begin{bmatrix}
0&-u_z&u_y\\
u_z&0&-u_x\\
-u_y&u_x&0
\end{bmatrix}
$$

则 $\mathbf u\times\mathbf v=[\mathbf u]_\times\mathbf v$，旋转矩阵为：

$$
R(\mathbf u,\theta)
=I\cos\theta
+(1-\cos\theta)\mathbf u\mathbf u^T
+\sin\theta[\mathbf u]_\times
$$

这个公式给出了轴角与旋转矩阵之间的直接联系。旋转矩阵可以直接作用于向量，但它用 9 个数表示 3 个自由度，6 个正交约束必须始终成立。数值积分会逐渐破坏这些约束，需要额外正交化。

### 6. 欧拉角适合描述姿态，但不适合无条件积分

欧拉角把一次三维旋转拆成三次有顺序的单轴旋转。采用 Z-Y-X 顺序时：

$$
R_b^n=R_z(\psi)R_y(\theta)R_x(\phi)
$$

$\phi$ 是 Roll，$\theta$ 是 Pitch，$\psi$ 是 Yaw。乘法顺序是定义的一部分，换一种顺序会得到另一套欧拉角。

欧拉角的优势是接近人的操作语言，例如“保持 Yaw 不变，让 Pitch 抬高 $10^\circ$”。它的限制来自参数化本身。当：

$$
\theta=\pm90^\circ
$$

Roll 和 Yaw 的瞬时旋转轴重合，两个自由度无法独立辨认，这个奇异点称为万向节锁。

陀螺仪三个分量也不能直接当作三个欧拉角导数。对于 Z-Y-X 欧拉角，机体系角速度满足：

$$
\begin{bmatrix}
\omega_x\\
\omega_y\\
\omega_z
\end{bmatrix}
=
\begin{bmatrix}
1&0&-\sin\theta\\
0&\cos\phi&\sin\phi\cos\theta\\
0&-\sin\phi&\cos\phi\cos\theta
\end{bmatrix}
\begin{bmatrix}
\dot\phi\\
\dot\theta\\
\dot\psi
\end{bmatrix}
$$

反解欧拉角导数时会出现 $1/\cos\theta$ 和 $\tan\theta$。当 $\theta$ 接近 $\pm90^\circ$ 时，这些项趋于无穷。姿态估计器通常用四元数连续积分，只在显示、交互或逐轴控制时转换为欧拉角。

### 7. 单位四元数怎样编码轴角旋转

二维平面上的单位复数：

$$
z=\cos\theta+i\sin\theta
$$

可以编码平面旋转。三维旋转还需要旋转轴。设任意空间向量 $\mathbf v$ 按右手定则绕单位轴 $\mathbf u$ 主动旋转 $\theta$，对应的单位四元数定义为：

$$
q=
\begin{bmatrix}
q_w\\
\mathbf q_v
\end{bmatrix}
=
\begin{bmatrix}
\cos(\theta/2)\\
\mathbf u\sin(\theta/2)
\end{bmatrix}
$$

$q_w$ 是标量部分，$\mathbf q_v=[q_x, q_y, q_z]^T$ 是向量部分。$\mathbf q_v$ 的方向给出旋转轴，长度是 $\sin(\theta/2)$。单位约束为：

$$
\lVert q\rVert^2
=q_w^2+q_x^2+q_y^2+q_z^2=1
$$

四个分量受到一个约束，留下三个自由度。$q$ 与 $-q$ 表示同一个物理旋转。

两个四元数 $p=[p_w,\mathbf p_v]^T$ 和 $q=[q_w,\mathbf q_v]^T$ 的 Hamilton 乘积为：

$$
p\otimes q=
\begin{bmatrix}
p_wq_w-\mathbf p_v^T\mathbf q_v\\
p_w\mathbf q_v+q_w\mathbf p_v+\mathbf p_v\times\mathbf q_v
\end{bmatrix}
$$

将三维向量写成纯四元数 $v_q=[0,\mathbf v]^T$，主动旋转后的向量为：

$$
v_q'=q\otimes v_q\otimes q^*
$$

其中共轭四元数为：

$$
q^*=
\begin{bmatrix}
q_w\\-\mathbf q_v
\end{bmatrix}
$$

若 $q$ 是单位四元数，则 $q^{-1}=q^*$。

### 8. 四元数可以从旋转矩阵推导

四元数旋转展开后对应矩阵：

$$
R(q)=
\begin{bmatrix}
1-2(q_y^2+q_z^2)&2(q_xq_y-q_wq_z)&2(q_xq_z+q_wq_y)\\
2(q_xq_y+q_wq_z)&1-2(q_x^2+q_z^2)&2(q_yq_z-q_wq_x)\\
2(q_xq_z-q_wq_y)&2(q_yq_z+q_wq_x)&1-2(q_x^2+q_y^2)
\end{bmatrix}
$$

把轴角四元数代入这个矩阵，并使用半角公式：

$$
1-\cos\theta=2\sin^2\frac\theta2
$$

$$
\sin\theta=2\sin\frac\theta2\cos\frac\theta2
$$

就会得到上一节的 Rodrigues 旋转矩阵。四元数中出现半角，是因为四元数旋转包含 $q$ 和 $q^*$ 两次乘法，展开后的乘积项组合成 $\sin\theta$ 与 $\cos\theta$。

若已知旋转矩阵 $R=[r_{ij}]$，它的迹满足：

$$
\operatorname{tr}(R)=1+2\cos\theta
$$

因此：

$$
q_w=\frac12\sqrt{1+\operatorname{tr}(R)}
$$

当 $q_w$ 不接近零时，其余分量可以计算为：

$$
q_x=\frac{r_{32}-r_{23}}{4q_w}
$$

$$
q_y=\frac{r_{13}-r_{31}}{4q_w}
$$

$$
q_z=\frac{r_{21}-r_{12}}{4q_w}
$$

当旋转角接近 $180^\circ$ 时，$q_w$ 接近零，直接相除会放大数值误差。实际转换算法通常根据最大的对角元素选择计算分支。

## 第三篇：用 IMU 估计姿态

### 9. 陀螺仪怎样更新四元数

在短时间 $\Delta t$ 内，若角速度近似不变，机体转过的旋转向量为：

$$
\Delta\boldsymbol\theta
=\boldsymbol\omega_{b/n}^b\Delta t
$$

对应的精确增量四元数为：

$$
\delta q=
\begin{bmatrix}
\cos(\lVert\boldsymbol\omega\rVert\Delta t/2)\\
\dfrac{\boldsymbol\omega}{\lVert\boldsymbol\omega\rVert}
\sin(\lVert\boldsymbol\omega\rVert\Delta t/2)
\end{bmatrix}
$$

角速度用当前机体系表达时，姿态采用右乘更新：

$$
q_{k+1}=q_k\otimes\delta q
$$

当 $\Delta t$ 很小时：

$$
\sin x\approx x,
\qquad
\cos x\approx1
$$

所以：

$$
\delta q\approx
\begin{bmatrix}
1\\
\frac12\boldsymbol\omega_{b/n}^b\Delta t
\end{bmatrix}
$$

由此得到四元数运动学方程：

$$
\dot q
=\frac12q\otimes
\begin{bmatrix}
0\\
\boldsymbol\omega_{b/n}^b
\end{bmatrix}
$$

最简单的离散积分是前向 Euler 方法：

$$
q_{k+1}\approx q_k+\dot q_k\Delta t
$$

浮点误差和离散近似会使四元数长度偏离 1，因此每轮更新后通常执行：

$$
q\leftarrow\frac{q}{\lVert q\rVert}
$$

归一化只修复四元数的长度约束。陀螺仪零偏仍会被持续积分成方向误差。

### 10. 加速度计怎样提供倾角参考

静止或低动态运动时，加速度计主要反映重力方向。根据所选符号约定，将测量转换成与重力预测方向一致的单位向量：

$$
\hat{\mathbf a}
=\frac{\mathbf a}{\lVert\mathbf a\rVert}
$$

当前四元数也能预测导航系重力单位向量 $\hat{\mathbf g}^n$ 在机体系中的方向：

$$
\hat{\mathbf g}^b(q)
=R_n^b(q)\hat{\mathbf g}^n
$$

现在有两个方向。$\hat{\mathbf a}$ 来自测量，$\hat{\mathbf g}^b(q)$ 来自姿态预测。两者重合时，估计的倾角与重力参考一致。两者存在夹角时，叉积给出旋转误差：

$$
\mathbf e
=\hat{\mathbf a}\times\hat{\mathbf g}^b(q)
$$

若夹角为 $\alpha$，两个向量都已归一化，则：

$$
\lVert\mathbf e\rVert=\sin\alpha
$$

小角度下：

$$
\sin\alpha\approx\alpha
$$

所以 $\mathbf e$ 的方向近似为姿态需要修正的旋转轴，长度近似为误差角。叉积的顺序与正负号取决于姿态定义和反馈写法，实现时必须通过单轴倾斜实验确认修正方向。

### 11. Mahony 反馈怎样抑制倾角漂移

Mahony 非线性互补滤波器将重力方向误差反馈到角速度通道。简化形式为：

$$
\dot{\hat{\mathbf b}}_g=-K_i\mathbf e
$$

$$
\boldsymbol\omega_c
=\boldsymbol\omega_m-\hat{\mathbf b}_g+K_p\mathbf e
$$

$$
\dot q
=\frac12q\otimes
\begin{bmatrix}
0\\
\boldsymbol\omega_c
\end{bmatrix}
$$

$K_p\mathbf e$ 提供即时修正。倾角误差越大，附加的纠正角速度越大；预测重力接近测量重力后，该修正随之减小。$\hat{\mathbf b}_g$ 是陀螺仪零偏估计，积分项利用长期存在的方向误差缓慢调整它。

不同实现也会把积分状态定义成直接相加的角速度补偿 $\mathbf i$：

$$
\dot{\mathbf i}=K_i\mathbf e
$$

$$
\boldsymbol\omega_c
=\boldsymbol\omega_m+\mathbf i+K_p\mathbf e
$$

这两种写法只是在积分状态的符号定义上不同。几何误差、反馈方向和四元数更新约定必须成套匹配。

Mahony 滤波可以从互补滤波角度理解。陀螺仪承担高频、快速的姿态变化，加速度计提供低频、长期稳定的竖直参考。它也可以从闭环误差系统理解：重力预测方向偏离测量方向后，比例反馈生成让误差减小的角速度，积分反馈估计造成持续误差的陀螺仪零偏。

### 12. 加速度参考何时不可信

云台平移加速、振动或受冲击时，加速度测量中包含运动分量。经过符号统一后，可概念性地写为：

$$
\mathbf a_m
=\mathbf g^b+\mathbf a_{\mathrm{motion}}^b+\mathbf n_a
$$

若把 $\mathbf a_{\mathrm{motion}}^b$ 当成重力，估计器会向错误方向修正。常见方法是设置加速度置信度 $c_a\in[0,1]$：

$$
K_p^{\mathrm{eff}}=c_aK_p
$$

$$
K_i^{\mathrm{eff}}=c_aK_i
$$

$c_a$ 可以根据加速度模长与 $g$ 的偏差、模长变化率、振动强度或运动状态计算。当 $c_a=0$ 时，估计器暂时只使用陀螺仪；当测量恢复可信后，再逐渐恢复重力反馈。直接硬切换增益可能造成姿态修正突变，实际系统常对置信度进行限速或低通处理。

### 13. 为什么重力无法修正 Yaw

重力只提供一条空间方向。把姿态绕重力轴旋转任意角度 $\psi$，重力向量保持不变：

$$
R_{\mathbf g}(\psi)\mathbf g=\mathbf g
$$

因此，加速度计只能约束 Roll 和 Pitch 对应的两个倾斜自由度，不能观测绕竖直轴的绝对 Yaw。没有外部航向参考时，Yaw 只能依靠陀螺仪积分，长期误差会随零偏积累。

若系统需要绝对 Yaw，可引入第二个不与重力平行的参考方向，例如地磁场、视觉特征、卫星航向、双天线定位或外部编码参考。两个不共线的参考向量可以确定完整的三维姿态。

### 14. 姿态初始化与数值实现

静止启动时，可直接由重力方向求初始 Roll 和 Pitch。某一常用符号约定下：

$$
\phi=\operatorname{atan2}(a_y, a_z)
$$

$$
\theta
=\operatorname{atan2}
\left(-a_x,\sqrt{a_y^2+a_z^2}\right)
$$

加速度无法提供 $\psi$，因此可将初始 Yaw 定义为 0，或用外部航向测量初始化。公式的正负号必须按实际坐标系和加速度符号调整。

离散实现还需要处理几项数值问题：

- 使用真实或准确的采样周期 $\Delta t$，周期误差会直接缩放积分角度。
- 每次四元数更新后归一化，避免长度约束逐渐漂移。
- 对加速度和反馈误差做适度滤波，滤波过强会引入相位滞后。
- 限制零偏积分状态，避免长时间错误参考造成积分饱和。
- 对接近零的向量禁止归一化，避免除零和噪声放大。

## 第四篇：从姿态误差到电机控制

### 15. 四元数怎样转换为欧拉角

采用前文的 Z-Y-X 顺序，单位四元数可转换为：

$$
\phi=\operatorname{atan2}
\left(2(q_wq_x+q_yq_z),1-2(q_x^2+q_y^2)\right)
$$

$$
\theta=\arcsin
\left(2(q_wq_y-q_zq_x)\right)
$$

$$
\psi=\operatorname{atan2}
\left(2(q_wq_z+q_xq_y),1-2(q_y^2+q_z^2)\right)
$$

计算 $\arcsin$ 前，应把输入限制在 $[-1,1]$，避免浮点舍入使其略微越界。四元数适合估计器内部连续更新，欧拉角适合显示、接收逐轴目标，以及工作范围远离奇异点的云台控制。

### 16. 姿态误差有两种常见表达

逐轴云台常直接计算欧拉角误差：

$$
e_\phi=\phi_d-\phi
$$

$$
e_\theta=\theta_d-\theta
$$

$$
e_\psi=\operatorname{wrap}(\psi_d-\psi)
$$

$\operatorname{wrap}$ 把周期角误差映射到 $(-180^\circ,180^\circ]$ 或 $(-\pi,\pi]$，让控制器选择较短旋转方向。这种方法直观，适合转角范围受限、轴间耦合较弱的云台。

大角度三维机动可直接定义四元数误差：

$$
q_e=q_d\otimes q^{-1}
$$

由于 $q_e$ 与 $-q_e$ 表示同一个旋转，通常选择标量部分非负的表示：

$$
q_e\leftarrow
\begin{cases}
q_e,&q_{e, w}\ge0\\
-q_e,&q_{e, w}<0
\end{cases}
$$

小误差时：

$$
q_e\approx
\begin{bmatrix}
1\\
\frac12\boldsymbol\theta_e
\end{bmatrix}
$$

因此旋转误差向量可以近似取为：

$$
\boldsymbol\theta_e\approx2\mathbf q_{e, v}
$$

它没有欧拉角在 $\theta=\pm90^\circ$ 处的参数奇异性，但仍要统一误差四元数的左右乘顺序和坐标表达系。

### 17. 串级 PID 怎样形成闭环

以某一个云台轴为例，角度外环为：

$$
e_\theta=\theta_d-\theta
$$

$$
\omega_d
=K_{p\theta}e_\theta
+K_{i\theta}\int e_\theta\,dt
+K_{d\theta}\frac{de_\theta}{dt}
$$

外环输出 $\omega_d$ 是目标角速度。角速度内环为：

$$
e_\omega=\omega_d-\omega
$$

$$
u
=K_{p\omega}e_\omega
+K_{i\omega}\int e_\omega\,dt
+K_{d\omega}\frac{de_\omega}{dt}
$$

$u$ 可以是目标电流、目标力矩或归一化驱动命令。电机产生力矩后，角速度先变化；速度变化再累积成角度变化。这与被控对象的物理积分链相匹配。

比例项根据当前误差立即响应。积分项累积长期误差，可抵消摩擦、重心偏置和恒定扰动力矩。微分项根据误差变化趋势增加阻尼，但会放大高频测量噪声。实际速度环常使用陀螺仪直接测得的角速度，角度环的微分项则可能省略，以免重复使用速度信息。

内环带宽通常高于外环。若内环足够快，外环可以近似认为目标角速度会被及时实现。两环带宽过于接近时，相位滞后和耦合会使调参困难。

### 18. 离散 PID、限幅和抗积分饱和

控制器运行在离散采样周期 $\Delta t$ 下。积分和微分可写为：

$$
I[k]=I[k-1]+e[k]\Delta t
$$

$$
D[k]=\frac{e[k]-e[k-1]}{\Delta t}
$$

离散 PID 输出为：

$$
u[k]=K_pe[k]+K_iI[k]+K_dD[k]
$$

执行器存在最大电流或最大力矩：

$$
u_{\mathrm{sat}}
=\operatorname{clip}(u, u_{\min}, u_{\max})
$$

若输出已经饱和，积分项仍继续增长，解除饱和后会造成明显过冲，这称为积分饱和。常见处理方法包括限制积分状态、饱和时停止同方向积分，以及使用回算：

$$
\dot I
=e+K_{aw}(u_{\mathrm{sat}}-u)
$$

其中 $K_{aw}$ 是抗饱和增益。目标角度也可经过速度限制或轨迹规划，避免一步命令要求执行器产生不可能实现的瞬时运动。

### 19. 机体角速度与关节角速度并不总相同

陀螺仪测得的是载荷的几何角速度向量 $\boldsymbol\omega^b$。串级控制器需要的却可能是每个机械关节绕自身转轴的角速度。设第 $i$ 个关节轴在机体系中的单位方向为 $\mathbf a_i^b$，沿该轴的角速度分量为：

$$
\omega_i=\left(\mathbf a_i^b\right)^T\boldsymbol\omega^b
$$

对于嵌套三轴机构，$\mathbf a_i^b$ 会随其他关节角变化，因此投影矩阵也是姿态或关节角的函数：

$$
\boldsymbol\omega_{joint}
=A(\boldsymbol\eta)\boldsymbol\omega^b
$$

$\boldsymbol\eta$ 表示关节角。机械轴接近共线时，$A$ 可能病态，某些方向上的运动需要多个关节共同完成。这是机械运动学耦合，与欧拉角参数化的万向节锁相关，但两者的分析对象不同：一个描述实际机构的轴布局，一个描述姿态坐标的数学奇异性。

### 20. Pitch 抬高 10 度时发生了什么

假设云台初始静止，目标从 $0^\circ$ 变为 $10^\circ$。不代入某一套具体增益，只观察各物理量的方向和时间顺序。

初始时：

$$
e_\theta=10^\circ-0^\circ=10^\circ
$$

角度外环产生正的目标角速度 $\omega_d$。此时实际角速度接近 0，所以速度误差为正：

$$
e_\omega=\omega_d-0>0
$$

速度环产生正力矩，云台获得正角加速度。随着实际角速度上升，$e_\omega$ 减小；随着实际角度接近 $10^\circ$，$e_\theta$ 也减小，外环给出的 $\omega_d$ 逐渐回到 0。

若云台因惯性越过目标，$e_\theta$ 变成负值，外环要求反向角速度，速度环产生制动力矩。阻尼不足时会出现多次往返振荡，积分过强时会增加过冲，输出限幅则决定最快可实现的角加速度。

姿态估计器在整个过程中并行工作。陀螺仪快速跟踪 Pitch 角速度，加速度计在动态较小的阶段提供竖直参考，四元数持续更新，再转换成控制器需要的姿态误差。

## 第五篇：把整条数据流连起来

### 21. 一个姿态估计周期

通用的姿态估计周期可以整理为：

```text
读取陀螺仪和加速度计
  -> 标定零偏、比例因子和轴不正交
  -> 将传感器分量转换到统一机体系
  -> 根据加速度状态计算重力参考置信度
  -> 用当前四元数预测机体系重力方向
  -> 用叉积构造方向误差
  -> 修正陀螺仪角速度和零偏估计
  -> 积分四元数并归一化
  -> 按需要输出欧拉角或旋转矩阵
```

坐标变换解决“每个数沿哪根轴”的问题，四元数积分解决“姿态怎样连续变化”的问题，重力反馈解决“倾角怎样避免长期漂移”的问题。

### 22. 一个云台控制周期

姿态估计结果进入控制器后，典型流程为：

```text
目标姿态
  -> 计算欧拉角误差或四元数误差
  -> 角度外环生成目标角速度
  -> 将几何角速度投影到机械关节轴
  -> 角速度内环生成目标力矩或电流
  -> 执行器限幅与抗积分饱和
  -> 电机和机械结构产生实际运动
  -> IMU 再次测量运动
```

姿态估计与控制通过传感器和机械对象闭合成反馈环。估计误差会进入控制误差，坐标方向错误会直接变成错误力矩，采样延迟会减少稳定裕度，因此坐标约定、时间同步和符号测试属于控制系统的一部分。

### 23. 理论模型的适用边界

本文采用的模型省略了柔性结构、齿隙、电机电气动态、传感器温漂、时间戳抖动和通信延迟。实际系统还可能遇到：

- IMU 安装矩阵不准确，导致轴间串扰。
- 加速度计受到持续平移加速，重力方向长期不可分离。
- 陀螺仪温度零偏变化，Yaw 漂移加快。
- 云台质量分布不平衡，产生随姿态变化的重力力矩。
- 电机饱和、摩擦和齿隙引入非线性。
- 控制周期和传感器采样不同步，引入额外相位延迟。

验证时可以从单轴开始：确认传感器正方向、姿态角正方向和电机正力矩方向构成负反馈，再检查六面静置、静止漂移、目标阶跃、外力扰动和大角度运动。三轴联调之前，单轴符号和单位必须闭合。

## 推荐资料

1. Joan Solà，*Quaternion Kinematics for the Error-State Kalman Filter*，2017。适合学习四元数乘法、局部角速度与离散积分。
2. Robert Mahony、Tarek Hamel、Jean-Michel Pflimlin，*Nonlinear Complementary Filters on the Special Orthogonal Group*，2008。适合学习方向误差、非线性互补滤波和零偏估计。
3. David M. Henderson，*Euler Angles, Quaternions, and Transformation Matrices for Space Shuttle Analysis*，1977。适合核对欧拉角顺序、旋转矩阵和四元数转换。
4. Karl Johan Åström、Richard M. Murray，*Feedback Systems: An Introduction for Scientists and Engineers*，2008。适合学习反馈稳定性、PID、饱和与离散控制。
5. John J. Craig，*Introduction to Robotics: Mechanics and Control*，2005。适合学习坐标变换、旋转表示和串联机构运动学。
