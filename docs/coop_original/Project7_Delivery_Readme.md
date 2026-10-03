# Project 7 三项任务交付总说明

版本：2026-10-02。依据用户截图锁定本版本设计参数，所有代码作为待本机 MATLAB/Simulink 验收的工程候选。

## 输出一览

| 任务 | 文件/程序包 | 配套说明 |
| --- | --- | --- |
| 1：K 与 Klqr | project7_q4_gains.m | project7_q4_gains.md |
| 2：MPC 手绘接口 | Project7_MPC_TrajectoryLab.zip | Project7_MPC_TrajectoryLab.md；包内 README、逐文件 .md |
| 3A：SF/LQR 噪声分析 | Project7_SF_LQR_Noise.zip | Project7_SF_LQR_Noise.md；包内 README、逐文件 .md |
| 3B：MPC 噪声分析 | Project7_MPC_Noise.zip | Project7_MPC_Noise.md；包内 README、逐文件 .md |

全部内容也在 Project7_Delivery_All.zip 内，三个工程文件夹已经展开，避免必须再解压嵌套 ZIP。逐文件 Markdown 说明每段代码的作用、原理、调用和边界，并附对应源码。

## 参数状态

本次截图确认的版本值：

- v0=0.5 m/s，固定名义 Wr=pi/4 rad/s。
- SF poles=[-1,-2,-4]。
- LQR/MPC Q=I3，R=1。
- 反馈误差 u=omega-omega_r；B=[0;0;-1]。

附加实现初值：Ts=0.1 s、N=20、omega 指令范围 ±1.2 rad/s、observer noise 标准差 [0.05 m,0.05 m,0.05 rad]、control noise 标准差 0.1 rad/s。这些是可调整的仿真配置，不自动升级为经过性能验证的最佳设置。

## 包之间如何同步

1. 任务1是设计参数和 K/Klqr 的来源。
2. 三个包自带相同版本的任务1和公共模块，因此无需跨 ZIP 调用。
3. MPC 对接包与 MPC 噪声包带相同 MPC 求解器；接口层不同，核心预测与优化一致。
4. SF/LQR 噪声包和 MPC 噪声包带相同 Q5 仿真循环、噪声生成、参考处理、指标和导出模块。
5. 所有复制模块从一份源代码统一生成，并在交付时检查哈希一致。之后如果只编辑某一份副本，其他包不会自动更新。
6. 用户原 COM_Project_T7.zip 没有被本次修改。运行 connect_TrajectoryLab 时，仅备份并替换所选择平台的 external 适配器；独立 launch 不修改平台文件。

## 两个容易混淆的实验环境

| 环境 | 状态/噪声来源 | 用途 |
| --- | --- | --- |
| 原 TrajectoryLab GUI + MPC adapter | 使用原平台 qHat/估计器；默认对接入口关闭平台噪声 | 核验手绘/预设路径与 MPC 接口 |
| 本次 Q5 噪声分析包 | 直接向 xe、ye、thetae 测量加噪声，向物理角速度加执行偏差 | 完成本次明确指定的 noise analysis |

本次没有把原平台的位姿状态增量噪声冒称为角速度噪声。Q5 噪声包默认不滤波，Qn/Rn 均为 []；另提供待填写协方差后才能运行的误差 KF 和 Qn/Rn 参数扫描。不同估计器与噪声定义的结果不能不加区分地放在同一对比表中。

## 推荐执行顺序

1. 先运行任务1得到 K、Klqr。
2. 在噪声包内运行 tests/run_project7_tests，查看完成与 SKIP 记录。
3. 用 duration=1~2 s、N=4~8 做短入口验收。
4. 接入已确认的 path 结构体或路径文件，再设正式时域。
5. 完成无噪声比较，再比较 observer、control、both。
6. 需要时执行单 pole、KF Qn/Rn 扫描。实际噪声和参考保持不变。
7. 最后再讨论速度失配与大初始误差；保留失败和限幅记录。

## 重要验证结果

32 个唯一 MATLAB 源文件通过语法树解析；独立数值核对复现 K/Klqr、验证 LTV 目标梯度、输入边界、误差变换和 Q5 噪声通道。但当前环境没有 MATLAB/Octave，尚未执行实际 MATLAB 工具箱、GUI、Simulink 或完整手绘路径性能实验。

特别是当前固定名义 LQR 对平台默认半径 3 m 圆的冻结线性模型有约 +0.002388 的实极点。名义点稳定不代表任意路径稳定。代码保留用户参数，未为了输出更好看而改动权重或增益。

本次已载入原 MPC ZIP 并恢复原五文件的职责分工，增加手绘接口和共享辅助模块；两套噪声包的 Kalman Qn/Rn 均留空，启用 KF 前须填写。默认 LTV MPC，可明确切换 nonlinear 预测；不能将两种结果混称。

详细数值与未完成验收见每包 VALIDATION.md。源资料及官方接口文档列在各包 README 中。
