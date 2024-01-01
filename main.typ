#import "template.typ": *

#show: project.with(
  title: "Design of DC Motor and Its Control System",
  authors: ("Junhao Li","Jinhang Wang","Yi Zhou","Mingyang Bao"),
  address: "School of Electric and Electrical Engineering, Huazhong University of Science and Technology, Wuhan, 430074, China",

  title_cn: "直流电机本体与控制系统设计",
  authors_cn: ("李俊昊","汪锦航","周燚","包铭阳"),
  address_cn: "华中科技大学电气与电子工程学院，武汉市 洪山区 430074",

  abstract: [
      This paper mainly describes the process of designing a DC motor and its control system, and provides a detailed introduction to the overall production and parameter testing methods of the DC motor, and explores the implementation of the DC motor analog and numerical control system.
  ],
  abstract_cn:[
      本文通过论述设计一台直流电机及其控制系统的过程，详细介绍了直流电机的整体制作思路和参数测试方法，探讨了直流电机控制系统的模拟实现与数字实现],

  keywords: ("DC Motor", "Control System", "PID Control", "Analog   
      Control", "Numerical Control"),
  keywords_cn:("直流电机", "控制系统", "PID控制", "模拟控制", "数字控制"),
  
  bibliography-file: "refs.bib",
)
//
//
//
= 引言
随着电子技术和自动控制技术的不断发展，工业自动化对宽调速、反应灵敏的驱动装备的需求日益增加。在这方面，直流电动机具有“得天独厚”的优越性，它拥有出色的调速性能，能够快速响应控制信号，实现精确的转速调节；有着理想的起动与制动特性，能够实现平稳启动和快速制动；同时还具有较大的过载能力，能够应对瞬时负载变化和高负载要求。因此，直流电动机被广泛应用于工业控制、航空航天、汽车产业、医疗设备和家用电器等各个领域。

PID（Proportional-Integral-Derivative）是一种经典控制算法，它通过比例、积分和微分三个方面对被控物理量进行精确控制，以实现设定值与实际值之间的误差最小化。比例控制用于调整响应速度，积分控制用于消除稳态误差，微分控制用于抑制系统震荡。由于其收敛速度快、稳定性好、实现简单，是目前直流电机转速控制的主流控制方法。

在本文中，我们团队设计了一台直流电机及其控制系统，并对其基本参数进行了测量。基于电机的基本参数，我们建立了电机的数学模型，并在控制系统中引入PID算法来实现对电机的闭环控制，包括模拟实现和数字实现两种方式。



= 电机本体设计及参数测试

== 电机本体设计
本次实验制作的直流电机，由一对永磁体励磁，转子由3D打印制成的8槽塑料模型作为“铁芯”，采用0.3mm漆包线对称绕制4个绕组，匝数为60匝，绕组端线焊接在对应换向器上。转子和换向器通过直径5mm钢杆固定在电机支架上，钢杆上有磁环与其同轴旋转，磁环与霍尔编码器配合实现电机速度与位置的测量。电机制作成品如图@img_motor 。
#figure(
    image("./img/motor.png"),
    caption: [电机成品图],
)<img_motor>

本次实验选用增量式正交编码器，电机旋转时，霍尔元件检测输出若干脉冲信号，输出两组相位相差90°的方波信号A与B。正转时，A相处在下降沿时，B相为高电平；A相处在上升沿时，B相则为低电平。反转时，A相处在下降沿时，B相为低电平。A相处在上升沿时，B相则为高电平。通过AB相关系，我们可以判断电机转向与转速。已知电机旋转一圈，编码器产生16个脉冲ppr，则转速满足公式@f_nm 与公式@f_wm 关系。
$
n_m = f_p / 16 times 60
$<f_nm>

$
omega_m = n_m times (2pi)/60
$<f_wm>


== 电机参数测试
影响电机运行的基本参数包括：绕组等效电阻$R_a$与等效电感$L$、机械时间常数$tau$、反电动势常数$K_b$、转矩常数$K_i$、空载稳态速度$w_(f&v)$、黏性摩擦系数$B_m$、转动惯量$J_m$。

我们需要测量电机的基本参数以建立电机控制模型。

=== 绕组等效电阻$R_a$与等效电感$L$
我们使用万用表直接测量绕组等效电阻$R_a$，利用LCR电桥设备直接测量等效电感$L$，图@img_RaL 为LCR电桥测量电感的其中一组数据，多次测量取平均值得到$R_a=2.133Omega$，$L=228.139mu&H$。\
#figure(
    image("./img/RaL.png"),
    caption: [LCR电桥测量等效电感],
)<img_RaL>

=== 机械时间常数$tau$
电机的机械时间常数$tau$定义为电机空载时，受到阶跃信号激励时，速度达到稳态值的63.2%对应的时间。我们用直流稳压电源模拟电机电枢电压的阶跃信号，用示波器记录电机速度上升波形，波形如图@img_tau 所示，得到机械时间常数$tau=0.26s$。
#figure(
    image("./img/tau.png"),
    caption: [机械时间常数测量],
)<img_tau>

=== 反电动势常数$K_b$及转矩常数$K_i$
我们将电机电枢电压$U$从8V逐次降低，记录对应电枢电流$I$和电机转速值$n$，反电势为$e_b=U-I&R_a$。我们绘制$E$-$n$曲线并线性拟合如图@img_eb 所示，根据公式@f_eb 得到反电动势常数$K_b=7.115 times 10^(-3)$。
$
e_b = K_b & w_m
$<f_eb>
#figure(
    image("./img/eb_curve.png"),
    caption: [反电势拟合曲线],
)<img_eb>
  
在理想情况下，反电动势常数$K_b$和电动机转矩常数$K_i$在SI单位中在数值上是相等的。因此，我们认为$K_i approx K_b = 7.115 times 10^(-3)$。

=== 空载稳态速度$w_(f&v)$及黏性摩擦系数$B_m$
对电机施加额定电压，根据稳态速度公式@f_wfv ，求得8V时电机稳态速度$w_(f&v)=824.67r&p&m$。
$
w_(f&v) = op("lim",limits:#true)_(t->oo) w(t) = (8K_i) / (K_i & K_b + B_m & R_a)
$<f_wfv>\

根据实验实测速度值，结合公式@f_bm，得到$B_m=8.638 times 10^(-6)$。
$
B_m = ((8K_i)/w_(f&v) - K_i & K_b) times 1 / R_a
$<f_bm>

=== 转动惯量$J_m$
电机的转动惯量可由公式@f_jm 计算，代入数据得到$J_m=8.417 times 10^(-6)$。
$
J_m = tau(B_m + (K_i & K_b) / R_a)
$<f_jm>\

至此，我们得到了直流电机的基本参数，如表@table_param 所示。
#figure(
    table(
    columns: (auto,auto,auto,auto),
    align: horizon,
    [*参数*], [*数值*], [*参数*], [*数值*],
    [等效电阻 $R_a$], [2.133 $Omega$ ], [等效电感 $L$], [228.139 $mu&H$],
    [时间常数 $tau$], [0.26 $s$ ], [反电势常数 $K_b$], [7.115e-3],
    [转矩常数 $K_i$], [7.115e-3], [粘性摩擦系数 $B_m$], [8.638e-6],
    [转动惯量 $J_m$], [8.417e-6], [空载稳态速度 $w_"fv"$], [824.67 $r&p&m$],
  ),
  caption: [直流电机基本参数],
)<table_param>

基于电机基本参数，我们在Simulink中建立电机的模型，如图@img_simulink 所示。

#figure(
    image("./img/simulink.png"),
    caption: [Simulink电机模型],
)<img_simulink>

\
= 控制系统的模拟实现
通过分析直流电机物理特性，我们得到经典直流电机开环控制模型，如图@img_open 所示，应用自动控制理论知识，我们画出直流电机闭环控制方框图，如图@img_system 所示。接下来我们使用Matlab提供的控制系统设计器Control System Designer对电机添加PID算法，并实现闭环控制。
#figure(
    image("./img/Model.png"),
    caption: [直流电机的开环数学模型],
)<img_open>

#figure(
    image("./img/Motor_Model.png"),
    caption: [直流电机闭环控制方框图],
)<img_system>

=== 速度单闭环
分析直流电机经典控制模型和闭环控制方框图，应用于Matlab提供的单闭环控制模型，我们得到速度单闭环的直流电机控制模型，图@img_matlab_single 为Matlab提供的单闭环控制模型，图@img_single 为推导出的速度单闭环电机控制模型。
#figure(
    image("./img/Matlab_SingleLoop_Model.png"),
    caption: [Matlab提供的单闭环模型],
)<img_matlab_single>

#figure(
    image("./img/My_SingleLoop_System.png"),
    caption: [推导出的电机单闭环模型],
)<img_single>

应用Matlab的Control System Designer，对模型添加一个积分器与一个实零点，对应传递函数为 $(s-a)/s$，相当于在控制系统中添加PI环节。调节根轨迹位置使阶跃响应波形满足设计需求，此时控制器传递函数为 $(1.4077s-1.673)/s$，对应PI参数为
$K_p = 1.4077$，$K_i = 1.673$，设计结果如图@img_single_locus 所示。
#figure(
    image("./img/single_locus.png"),
    caption: [根轨迹法调节单闭环控制参数],
)<img_single_locus>

Simulink仿真结果如图@img_single_sim 所示。
#figure(
    image("./img/single_sim.png"),
    caption: [单闭环仿真结果],
)<img_single_sim>

=== 速度电流双闭环
分析直流电机经典控制模型和闭环控制方框图，结合公式@f_G1 和@f_G2 ，应用于Matlab提供的双闭环控制模型，我们得到速度电流双闭环的直流电机控制模型，图@img_matlab_double 为Matlab提供的双闭环控制模型，图@img_double 为推导出的速度电流双闭环电机控制模型。
$
G_1(s) = I(s) / V_a(s) = 1 / (L & s + R_a)
$<f_G1>

$
G_2(s) = Omega(s) / I(s) = K_i / (J & s + B_m)
$<f_G2>

#figure(
    image("./img/Matlab_DoubleLoop_Model.png"),
    caption: [Matlab提供的双闭环模型],
)<img_matlab_double>

#figure(
    image("./img/My_DoubleLoop_System.png"),
    caption: [推导出的电机双闭环模型],
)<img_double>

然而，由于模拟控制的电路板中，双闭环的内环只有PID中的比例环节，我们只对模型中的控制器修改其比例参数，由自动控制理论可知，0型系统（无微分环节系统）对阶跃信号的稳态误差$e_(s&s)$ 遵循公式@f_ess ，当$K_p$增大时，$e_(s&s)$随之减小。
$
e_(s&s) = 1 / (1 + K_v)
$<f_ess>\

我们取电流内环$K_p=1000$，得到稳态响应为$0.998$，稳态误差$0.2%$，满足设计需求，设计结果如图@img_double_locus_in 所示。
#figure(
    image("./img/double_locus_in.png"),
    caption: [根轨迹法调节双闭环内环控制参数],
)<img_double_locus_in>

对于速度外环，受控装置前馈通道传递函数由原先开环传递函数改为添加了电流内环的闭环传递函数，再次通过添加积分器与实零点在控制系统中添加PI环节。调节根轨迹位置使阶跃响应波形满足设计需求，此时控制器传递函数为 $(0.68407s-0.7156)/s$，对应PI参数为
$K_p = 0.6841$，$K_i = 0.7156$，设计结果如图@img_double_locus_out 所示。
#figure(
    image("./img/double_locus_out.png"),
    caption: [根轨迹法调节双闭环外环控制参数],
)<img_double_locus_out>

== 模拟控制实际效果
本文实验中，使用模拟控制电路板如图@img_ana_circuit 所示，其包括一个拥有比例环节P的电流内环和一个拥有比例积分环节PI的速度外环，可以通过开关设置“开环”、“单环”、“双环”三种工作模式。
#figure(
    image("./img/ana_circuit.png"),
    caption: [根轨迹法调节双闭环外环控制参数],
)<img_ana_circuit>


其中，电流内环的比例环节P的参数由电阻$R_40$、$R_26$、$R_37$、$R_29$决定，满足公式@in_kp ；速度外环的比例积分环节PI的参数由电阻$R_10$、$R_11$、$R_6$、$R_14$和电容$C_5$共同决定，满足公式@out_kp 及@out_ki。
$
K_p' = (R_40 & R_26) / (R_37 & R_29)
$<in_kp>

$
K_p = (R_10 & R_11) / (R_6 & R_14)
$<out_kp>

$
K_i = R_10 / (R_6 & R_14 & C_5)
$<out_ki>\

受实际电阻误差影响，我们的电阻值为: $R_40=5k Omega$, $R_26=298.3 Omega$, $R_37=3.3k Omega$, $R_10=5k Omega$, $R_29=198.1k Omega$, $R_11=81.3k Omega$, $R_6=3.3k Omega$, $R_14=89.2k Omega$。对应的$K_p=1.3810$, $K_i=1.698$, $K_p'=1006$。

实际测试中，电机开环运行时，80%上升时间为1060ms；单环运行时，80%上升时间为320ms，超调量23%，稳态误差5%，5%调节时间1.32s；双环运行时，80%上升时间为220ms，超调量47%，稳态误差3.5%，5%调节时间1.12s。实测波形如图@img_ana_wave 所示。
#figure(
    image("./img/ana_wave.png"),
    caption: [模拟控制实测波形],
)<img_ana_wave>

由结果我们可以看到，加入PID实现闭环控制时，电机的响应速度大幅度提升，调速的效果也有所改善，但是也带来了超调的问题，近50%的超调量在实际电机运行中显然是危险的，本次实验所得结果还需进一步修正。

= 控制系统的数字实现
结合电机学、自动控制原理以及嵌入式系统课程中学习的理论知识，我们对直流电机实现数字控制。

数字控制部分模块主要包括PWM产生、编码器测速、定时器中断、模式控制、串口通讯、ADC采样、PID控制程序。

== 数字控制代码及成果
Matlab仿真流程与建模流程同模拟部分，在此不做过多赘述，数字部分主要通过代码与参数调整以达到所预期的效果。

=== 速度单闭环
我们利用PID控制程序对转速给定PI参数，设定比例系数kp=3.5，比例积分系数ki=3.5。在转速单闭环控制中，我们通过调节电压参考值来改变我们所预期的转速，并根据转速实际值与参考值进行PI运算来给定占空比，从而使电机达到我们所期望的转速。

主要代码如下：
```rust
PID_Init(&speedPIDStructure,3.5, 3.5, 0, 0, 0.001,-0.98, 0.98, 0);
speedPIDStructure.ref = (refVoltage - 1.65f) * 3409.f;
duty = PID_Calc(&speedPIDStructure, speed);
```
如图@img_num_single 所示，蓝色线为期望转速，黄色线为实际转速，在所整定的PI参数控制下，直流电机的速度成功完成了跟随，并且具有良好的性能指标。
#figure(
    image("./img/num_single.png"),
    caption: [数字单闭环控制波形],
)<img_num_single>

=== 速度电流双闭环
我们利用PID控制程序对转速和电流给定PI参数，设定转速环kp=3.5，ki=3.5；电流环kp=0.5，ki=0.8。通过代码可知，速度单闭环与速度电流双闭环均选择转速作为参考基准，但与单闭环不同的是，双闭环对外环转速PI得到结果又进行了电流的PI运算,从而使电流环作为速度环的内环，进而根据电流环PI运算的结果来改变占空比，从而实现了速度电流的双闭环。

主要代码如下：
```rust
PID_Init(&speedPIDStructure,3.5, 3.5, 0, 0, 0.001,-1000, 1000, 0);
PID_Init(&IPIDStructure,0.5, 0.8, 0, 0, 0.001,-0.98, 0.98,0);
speedPIDStructure.ref = (refVoltage - 1.65f) * 3409.f;
IPIDStructure.ref = PID_Calc(&speedPIDStructure, speed);
duty = PID_Calc(&IPIDStructure, Ivalue);
```
如图@img_num_double 所示，蓝色线为期望转速，黄色线为实际转速，在速度电流双闭环的PI控制下，直流电机的速度与电流均成功闭环，同时具有良好的性能指标。
#figure(
    image("./img/num_double.png"),
    caption: [数字双闭环控制波形],
)<img_num_double>

=== 速度电流位置三闭环
我们利用PID控制程序对转速、电流、位置给定PI参数，设定转速环kp=3.5，ki=3.5；电流环kp=0.5，ki=0.8，位置环kp=20，ki=25。不同于单闭环和双闭环，三闭环选择位置作为参考，即位置环为最外环，通过改变外加电压，即可改变所期望的位置，速度环为中间环，电流环为最内环，通过位置环PI给定速度，再根据速度环PI给定电流，最后再通过电流环PI得出占空比，从而完成闭环控制。

主要代码如下：
```rust
PID_Init(&speedPIDStructure,-3.5, -3.5, 0, 0, 0.001,-1000, 1000, 0);
PID_Init(&IPIDStructure,-0.5, -0.8, 0, 0, 0.001,-0.98, 0.98,0);
PID_Init(&positionPIDStructure,20, 25, 0, 0, 0.001,-5000, 5000, 0);
positionPIDStructure.ref = (refVoltage - 1.65f) * 5000.f;
speedPIDStructure.ref = PID_Calc(&positionPIDStructure, position);
IPIDStructure.ref = PID_Calc(&speedPIDStructure, speed);
duty = PID_Calc(&IPIDStructure, Ivalue);
```
如图@img_num_trible 所示，绿色线为期望位置，粉色线为实际位置。可以看到，期望位置改变后，实际位置进行了平稳的跟随改变，说明系统的性能良好，符合预期。
#figure(
    image("./img/num_trible.png"),
    caption: [数字三闭环控制波形],
)<img_num_trible>

== 数字控制的PI参数整定
PI参数的整定在完成数字控制的过程中至关重要，如果没有合适的PI参数，系统的闭环性能会很差，甚至根本无法完成闭环。在整定PI参数的过程中，我们主要应用了以下俩种方式。

=== Control System Designer
应用Matlab的Control System Designer,可以准确的得到所需要的PI参数，具体方式已在模拟部分说明，此处不做过多赘述。

=== 根据vofa+显示波形手动调参
数字控制程序中，通过串口通讯功能将所关注的参数输出到串口，通过上位机vofa+可以观察其闭环性能指标，如调节时间、超调量、稳态误差等时域指标，再根据《自动控制原理》所学的知识，手动的改变PI参数，不断的修正，最终得到我们所预期的结果。这也是我们在本次数字控制实验中主要所采用的方法。

= 总结
本次实验，我们通过制作一台直流电机，并基于传统根轨迹法与现代数学工具，设计其控制系统实现模拟与数字的闭环控制，深入探讨了直流电机本体与控制系统的设计思路与方式方法。让理论知识与工程实践相互促进。

= 致谢
本文中实验方案的制定和实验数据的测量记录工作是在华中科技大学教师吴葛、助教陈家睿等工作人员的大力支持下完成的，在此向他们表示衷心的感谢。