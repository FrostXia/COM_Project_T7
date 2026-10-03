# validate.m 逐段说明

**作用：**在运行前拒绝维度、单位类别、噪声和协方差设置错误。

**调用关系：**由 run_cases 和 simulate_q5 调用。

**边界：**仅检查输入结构合理，不是闭环性能证明。

代码中的 %% 是主要逻辑分段；下面按执行顺序解释。短辅助函数没有强行拆成多段。完整源码附在末尾，便于逐项核对。

### 第 1 段：实现与原理

检查单个速度、Ts、N、仿真时长、三维初始位姿。

### 第 2 段：实现与原理

检查标准差非负、随机种子范围、输入上下限。

### 第 3 段：实现与原理

none 模式允许 Qn/Rn 留空；kf 模式先检查是否填写，再检查标量 Qn 和 3×3 Rn，缺失时抛出 P7:KalmanCovarianceUnset。

## 验证状态

MATLAB 语法树解析通过；相关数学使用 Python/SciPy 作了独立核对。没有在交付环境执行 MATLAB 或 Simulink，具体运行检查见包内 VALIDATION.md。对原平台复制函数，本次只做源码、调用和语法核对，未将原作者的静态记录当作本次运行结果。

## 对应源码

~~~matlab
function validate(c)
% VALIDATE Check shared simulation inputs before a run.
validateattributes(c.v0,{'double'},{'scalar','finite','positive'});
validateattributes(c.vr,{'double'},{'scalar','finite','positive'});
validateattributes(c.Ts,{'double'},{'scalar','finite','positive'});
validateattributes(c.N,{'double'},{'scalar','integer','positive'});
validateattributes(c.duration,{'double'},{'scalar','finite','>=',c.Ts});
validateattributes(c.integrationSteps,{'double'},{'scalar','integer','positive'});
validateattributes(c.q0,{'double'},{'size',[3 1],'finite','real'});
validateattributes(c.qr0,{'double'},{'size',[3 1],'finite','real'});
validateattributes(c.measurementStd,{'double'},{'size',[3 1],'finite','nonnegative'});
validateattributes(c.controlStd,{'double'},{'scalar','finite','nonnegative'});
validateattributes(c.seed,{'double'},{'scalar','integer','>=',0,'<',2^32-2});
assert(isfinite(c.wMin) && isfinite(c.wMax) && c.wMin<0 && c.wMax>0,'P7:Bounds','Bounds must straddle zero.');
assert(ismember(c.filter.mode,{'none','kf'}),'P7:Filter','Filter must be none or kf.');
assert(ismember(c.mpc.mode,{'ltv','nonlinear'}),'P7:MPC','Unknown MPC model.');
validateattributes(c.analysisStart,{'double'},{'scalar','finite','nonnegative'});
if strcmp(c.filter.mode,'kf')
    assert(~isempty(c.filter.Qn) && ~isempty(c.filter.Rn), ...
        'P7:KalmanCovarianceUnset', ...
        'Fill c.filter.Qn (scalar) and c.filter.Rn (3-by-3) before enabling KF.');
    validateattributes(c.filter.Qn,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(c.filter.qScale,{'double'},{'scalar','real','finite','positive'});
    validateattributes(c.filter.rScale,{'double'},{'scalar','real','finite','positive'});
    validateattributes(c.filter.initialMean,{'double'},{'size',[3 1],'finite','real'});
    for name={'Rn','P0'}
        P=c.filter.(name{1});
        assert(isequal(size(P),[3 3]) && isreal(P) && all(isfinite(P(:))) && ...
            norm(P-P','fro')<1e-10 && min(eig(P))>=0,'P7:Covariance','Invalid covariance.');
    end
    assert(min(eig(c.filter.Rn))>0,'P7:Covariance','Measurement covariance must be positive definite.');
end
end
~~~
