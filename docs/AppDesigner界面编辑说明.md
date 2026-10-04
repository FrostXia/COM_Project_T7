# App Designer 界面编辑说明

Step 2 的 Desired Duration 是只读显示控件，Tag 为 `view_ui_desiredDuration`。数值由处理后路径长度除以 Desired Trajectory Speed 自动计算；不要增加独立时长输入。仿真到时结束，不延长时间等待终点。

## 入口与文件分工

在 MATLAB 中将当前文件夹设为项目根目录。正常运行仍使用 `start_app`。可视化编辑使用以下命令：

```matlab
edit_interface          % Open the main view
edit_interface('path')  % Open the trajectory editor
edit_interface('noise') % Open the noise dialog
```

| 文件 | 负责的界面 |
| --- | --- |
| `Trajectory/designer/TrajectoryDesignerApp.mlapp` | 三步流程、控制器参数、配置摘要、结果图与状态区 |
| `Trajectory/designer/TrajectoryEditorApp.mlapp` | 预设路径、手绘、平滑、路径预览和导入 |
| `Trajectory/designer/TrajectoryNoiseApp.mlapp` | Measurement / Control / State Noise 开关与标准差 |
| `Trajectory/src/TrajectoryAppController.m` | 路径处理、步骤切换、仿真调用、图表更新、导出等行为 |
| `Trajectory/src/bind_designer_views.m` | 将已保存的可视化控件连接到现有行为 |
| `Trajectory/src/attach_designer_controller.m` | 主界面启动时创建两个子窗口并连接后端 |

三个 `.mlapp` 都包含真实的 App Designer 控件，可在 **Design View** 选中和编辑。它们不是只调用旧窗口的空壳。原有 `controller`、`adapter`、`utils` 分工保持不变。

## 如何调整布局

1. 打开相应文件，选择 **Design View**。
2. 在画布或右侧 **Component Browser** 中选中按钮、标签、输入框或坐标轴。
3. 在属性面板修改静态 `Text`、字体、对齐等属性。窗口较小时可以最大化 App Designer，或缩小画布显示比例。
4. 界面采用 Grid Layout。调整控件所在行列，以及父布局的 `RowHeight`、`ColumnWidth`、`Padding`、`RowSpacing`、`ColumnSpacing`；控件尺寸主要由网格决定。
5. `Ctrl+S` 保存，重新运行 `start_app` 检查实际效果。

主界面的 `Step1`、`Step2`、`Step3` 面板共用同一位置。编辑其他步骤时，在组件树中选中目标面板；必要时临时关闭其他面板的 `Visible` 并打开目标面板。运行程序时会统一恢复到 Step 1，不改变原来的三步操作流程。

噪声窗口与路径窗口应通过主程序测试。直接运行这两个子 `.mlapp` 只能预览布局；它们的按钮行为由主程序启动时连接。

## 哪些内容仍在代码修改

- **保留控件的 `Tag`**，例如 `view_ui_run`、`view_MainAx`。运行时按 Tag 查找控件；修改或重复 Tag 会导致绑定失败。组件的 Code Name 可以调整，但应保留主窗口 `UIFigure` 和 `Controller` 属性。
- 按钮、数值框等回调在 `bind_designer_views.m` 中连接。需要改变行为时，修改该文件和 `TrajectoryAppController.m`，不要给同一控件另加一套冲突回调。
- 运行时的数值默认值仍来自 `utils/default_config.m`，包括预设形状、噪声、速度及控制器配置。仅修改 `.mlapp` 中的初始 Value 不会覆盖这些配置。
- 步骤标题、配置摘要的文字和值、绘图标题与图例由 `TrajectoryAppController.m` 动态更新。静态按钮和标签文字可以在 Design View 中修改。
- 绘图曲线的颜色与线型仍由绘图代码控制；`utils/trajectory_plot_colors.m` 提供公共色表。
- Step 3 的摘要预留 18 对标签，运行时更新文字和值，并隐藏未用行。字体和布局可视化编辑；不要删除这些已绑定标签。

程序化备用布局已删除。`TrajectoryAppController` 必须接收三个原生视图，由启动器创建；请使用 `start_app`，不要直接构造 `TrajectoryAppController(root)`。布局只编辑 `.mlapp`，无需重新运行旧迁移脚本。Nominal Omega R 在 State Feedback/LQR 下可编辑，默认按路径计算；重新处理路径会恢复自动值。

## 仿真和导出

迁移继续复用原来的 Simulink 模型、控制器、状态估计与噪声实现。运行结果仍导出两份带时间戳的 CSV：平滑后的 Desired Trajectory 和当前实验的 Evaluation。

Evaluation 的 Value 列包含文本、标量和矩阵。MATLAB 读取时应显式指定字符串列，避免自动推断跳过实验信息或将文本变成 NaN：

```matlab
options = delimitedTextImportOptions('NumVariables',3,'Delimiter',',', ...
    'DataLines',[2 Inf],'VariableNames',{'Metric','Value','Unit'}, ...
    'VariableTypes',{'string','string','string'});
evaluation = readtable('evaluation_TIMESTAMP.csv',options);
```

Trajectory CSV 仍是无表头 XY 两列，可通过 Import Existing Trajectory 重新导入。

## 验证范围

迁移在 MATLAB R2025a 中生成。验证包括三个文件的设计时组件加载、App Designer 原生保存、主程序初始化、三种预设路径、手绘样本平滑、四种控制器选择、噪声参数应用、短时 Open Loop Simulink 运行，以及导出路径重新读取。

这些检查用于确认界面迁移没有切断现有功能，不代表已完成全部控制算法的性能评估。

可重复运行：

```matlab
setup_paths
test_designer_views(true)
```
