# confun_Project7_MPC.m 逐段说明

**作用：**保留参考案例约束函数的位置，说明当前为何没有额外等式约束。

**调用关系：**p7_mpc_error_step 作为 fmincon 的 nonlcon 参数调用。

**边界：**未实现障碍物、状态边界或转向变化率约束。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

当前决策变量仅为输入；预测状态已由模型递推，因此不再重复施加动力学等式。

### 第 2 段：实现与原理

物理角速度约束已转换为 u 的 lb/ub。

### 第 3 段：实现与原理

如果未来将状态也设为独立决策变量，必须新增初值和动力学等式，不能继续保留空 ceq。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function [c,ceq]=confun_Project7_MPC(~)
% CONFUN_PROJECT7_MPC States are rolled out inside the objective.
% Physical angular-rate bounds are represented by lb/ub in the solver.
c=[]; ceq=[];
end
~~~
