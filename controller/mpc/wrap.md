# wrap.m 逐段说明

**作用：**保留原 ZIP 的角度归一化辅助文件，输出 [-pi,pi]。

**调用关系：**compute_next_pose_Project7 调用；与原模板 while 实现采用相同角度单位。

**边界：**这是角度表示转换，不是角速度限幅。角速度输入单位仍为 rad/s。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

用模运算消除整数个 2*pi，可处理标量和数组；正奇数倍 pi 保留为 +pi。

### 第 2 段：实现与原理

只规范实际位姿表示，不给 MPC 预测误差施加硬限制，也不在优化中截断误差。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function nu=wrap(alpha)
% WRAP Angles in radians mapped to [-pi,pi], including vector inputs.
validateattributes(alpha,{'double'},{'real','finite'});
nu=mod(alpha+pi,2*pi)-pi;
nu(nu==-pi & alpha>0)=pi;
end
~~~
