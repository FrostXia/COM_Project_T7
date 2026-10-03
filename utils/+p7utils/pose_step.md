# pose_step.m 逐段说明

**作用：**复现 Q5 compute_next_pose 的 100 小步积分顺序。

**调用关系：**实际机器人和 q4circle 无噪声参考均调用该积分器。

**边界：**不是 TrajectoryLab 中的精确零阶保持积分器；需区分数值积分差异。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

每个小步先更新航向，再使用新航向计算 x、y。

### 第 2 段：实现与原理

本包让全局航向连续积累，误差计算时再 wrap；位置运动与原模板的周期航向表示等价。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function qnext=pose_step(q,v,w,Ts,n)
% POSE_STEP The Q5 small-step ordering: update heading before x and y.
h=Ts/n;
qnext=q;
for i=1:n
    qnext(3)=qnext(3)+w*h;
    qnext(1)=qnext(1)+v*cos(qnext(3))*h;
    qnext(2)=qnext(2)+v*sin(qnext(3))*h;
end
% Keep global heading continuous. Only differences and innovations are wrapped.
end
~~~
