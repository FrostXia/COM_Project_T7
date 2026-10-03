# MPC 原 ZIP 架构对照与本次修订

依据：本次上传的 `Project7_MPC_Template(1).zip`，已读取其中全部 5 个 MATLAB 文件。当前版本保留原文件名和职责，并将公共计算抽为辅助函数，供独立入口、手绘平台、MPC 噪声实验复用。

## 五个原文件一一对应

| 原 ZIP 文件 | 新包位置 | 原职责与本次处理 |
| --- | --- | --- |
| launch_Project7_MPC.m | 根目录同名文件 | 保留“参数/参考 → for 循环 → 计算误差 → 预测 N 步 → 优化 → 执行首项 → 推进实际位姿”。由脚本改为函数，允许传入配置及路径；参考加载由它调用 reference_data 完成。 |
| objfun_Project7_MPC.m | mpc/ 同名文件 | 仍对 N 个角速度修正量 U 优化，内部预测误差并累加 Q/R 代价；用显式 e0、p 替代 global，增加 LTV 梯度。 |
| compute_next_pose_Project7.m | mpc/ 同名文件 | 恢复实际全局位姿推进职责，保留 `(theta,x,y,v,omega,dt)` 输入与 `(theta_next,x_next,y_next)` 输出；不再混称为误差预测函数。 |
| confun_Project7_MPC.m | mpc/ 同名文件 | 仍返回空 c、ceq，保留扩展位置；本版的物理角速度边界通过 fmincon 的 lb/ub 表达。 |
| wrap.m | mpc/ 同名文件 | 保留角度归一化职责，输出 [-pi,pi]；模运算替代逐次 while，兼容数组。它不是角速度约束。 |

不复制原脚本中的 `clear`，避免清除调用者配置。MPC 辅助文件位于 mpc/，入口会加入所需路径；因此目录较原五个文件平铺有所扩展。

## 辅助模块为什么增加

| 文件 | 职责与调用关系 |
| --- | --- |
| p7_mpc_error_step.m | 将主程序里的 fmincon 调用、输入约束、预测矩阵、热启动、解检查集中管理；三个 MPC 使用入口共享此函数。 |
| predict_error_Project7.m | 由 objfun 调用，递推 LTV 或可选非线性误差状态；与实际机器人位姿推进分开。 |
| project7_mpc_step.m | 将 TrajectoryLab 的 qHat/ref 接口转换为误差，返回物理角速度。 |
| connect_TrajectoryLab.m | 启动原手绘 GUI，备份并安装 external 适配器；不承担独立 MATLAB 主循环。 |
| bridge/p7_external_controller.m | 平台 external 插槽调用 project7_mpc_step。 |
| +p7core/reference_data.m | 一次加载 path/MAT/XY DAT/CSV 或五列时间表，并准备未来参考窗口。objfun 不读取文件。 |
| +p7core/simulate_q5.m | 噪声实验保留 Q5 循环和注噪位置，调用相同 MPC 求解器；避免在两处重复控制算法。 |

## 参数及数学差异明确记录

- 原 ZIP：Ts=0.2、N=10、Q=diag([10,10,5])、R=0.5，临时直线参考。当前版延续前次交付 Ts=0.1、N=20，以及用户已确认 Q=eye(3)、R=1；不因新上传旧模板而改回旧权重。v0=0.5 m/s 保留。
- 原 objfun：一阶 Euler `Ad=I+A*Ts, Bd=B*Ts`。当前默认仍为沿参考角速度变化的线性预测，但用增广 expm 离散化，允许显式速度失配项；可选 nonlinear 模式需要在报告区分。
- 固定 pi/4 的名义 Wr 用于 SF/LQR 设计。MPC 每拍仍使用路径的未来真实 wr 序列，不用名义值覆盖它。
- 原实际运动函数是单步 Euler；当前采用 Q5 的 100 小步、先航向后位置更新，便于无噪声与噪声实验比较。独立 launch 固定 integrationSteps=100 并进行检查。GUI 仍使用原平台运动积分，数值轨迹不必逐点相同。
- 原代价求和在当前版每项乘 Ts；默认终端权重为零，同一 Ts 下这是统一正比例缩放，单独此缩放不改变最优输入。默认没有终端集合或稳定性保证。
- 物理指令为 omega=wr+u。限制物理 omega 时，优化边界是 wMin-wr ≤ u ≤ wMax-wr，不能直接把同一上下限套给 u。
- 原模板每次以全零输入启动优化；当前将上一拍物理输入序列平移，减去新的参考序列，再投影到边界，形成热启动。
- 参考末端的预测窗口沿末端切线延伸，实验在路径结束处停止；不再通过 `length(time)-N` 提前丢弃末尾整段。
- 默认 Q4 圆是接口验收用参考；实际手绘路径由第二参数传入，不依赖圆的显式 desired function。

## 入口变化与兼容

```matlab
% 独立无噪声 MPC：主程序负责轨迹载入和循环
c = p7core.defaults;
c.duration = 2; c.N = 8; c.save = false;
result = launch_Project7_MPC(c);  % 默认名义圆
% result = launch_Project7_MPC(c,'trajectory.dat');

% 手绘 GUI 对接入口（选择原 TrajectoryLab 文件夹）
app = connect_TrajectoryLab;
```

`launch_Project7_MPC(平台目录)` 的旧用法仍会转到 GUI 连接入口。无参数 `launch_Project7_MPC` 则运行独立 MPC。`run_mpc_example(c,inputPath)` 为新主入口的别名，返回单次 result 结构；上一版的 runs 单元数组返回格式不再使用。噪声入口 run_mpc_noise / run_sf_lqr_noise 的输入输出格式保留。

## 两套噪声包的 Kalman 参数

```matlab
c.filter.mode = 'none';
c.filter.Qn = [];  % 待填写：角速度扰动的标量方差
c.filter.Rn = [];  % 待填写：三个误差测量的 3×3 协方差
```

这两个空值并非“噪声为零”。实际注噪仍由 measurementStd 和 controlStd 控制，沿用 Q5 初值；不会用它们自动补齐 KF 参数。需要 KF 的实验或三维扫描，必须先填写 Qn/Rn，否则抛出 `P7:KalmanCovarianceUnset`。本版将旧字段 QnOmega 统一命名为 Qn，若使用上一版保存的配置结构，应重新取 defaults 并迁移明确需要保留的字段。

这里只在角速度通道定义标量 Qn；进入三维离散状态预测的协方差为 `Bd*Qn*Bd'`。Rn 为测量 xe、ye、thetae 的 3×3 矩阵。过程噪声与测量噪声的互协方差按独立假设为零。控制代价 Q/R 与上述 KF 协方差是不同参数，未因本次留空而改动。

## 验证界限

本次完成源文件逐项对照、语法树解析、数学独立核对、复制一致性与 ZIP 完整性检查。新增 MATLAB 入口/角度函数/KF 留空检查写入 tests/run_project7_tests.m，须在 MATLAB 上执行；当前环境没有 MATLAB，未宣称 GUI 或真实 fmincon 仿真通过。
