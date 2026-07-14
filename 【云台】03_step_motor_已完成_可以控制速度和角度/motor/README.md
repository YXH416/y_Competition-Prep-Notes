# 双轴步进电机云台工程说明

本工程基于第一版 Python 视觉识别程序继续开发，使用 MSPM0G3507 控制双轴步进电机云台。当前代码已完成步进电机 1、2 的初始化和基础函数调用，支持速度与角度控制，并加入 K230 目标数据解析函数及云台闭环控制模板。

## 当前实现

### 步进电机 1、2 初始化

`main.c` 在 `SYSCFG_DL_init()` 之后调用 `step_motor_init()`。该函数同时完成两路电机驱动的 RST、SLP、DIR、DCY 引脚初始化，清零剩余步数和运行状态，并使能两路 PWM 定时器中断。

- 电机 1：默认作为水平/Yaw 轴，编号 `STEP_MOTOR_1`。
- 电机 2：默认作为俯仰/Pitch 轴，编号 `STEP_MOTOR_2`。
- 默认每个脉冲对应 `0.05625°`，实际值需与驱动器细分设置一致。

### 常用函数调用

```c
SYSCFG_DL_init();
step_motor_init();

/* 分别设置电机 1、2 的转速，单位为度/秒。 */
step_set_speed(90U, STEP_MOTOR_1);
step_set_speed(90U, STEP_MOTOR_2);

/* 设置方向后启停电机。 */
step_motor_dir_set(1U, STEP_MOTOR_1);
step_motor_start(STEP_MOTOR_1);
step_motor_stop(STEP_MOTOR_1);

/* 相对角度/有符号步数控制，调用后由中断非阻塞计步。 */
step_motor_move_relative(10.0f, STEP_MOTOR_1);
step_motor_move_steps(-200, STEP_MOTOR_2);
```

保留的 `step_motor_set_angle()` 是旧版无符号角度接口，它沿用 `step_motor_dir_set()` 预先选择的方向。云台闭环优先使用 `step_motor_move_steps()` 或 `step_motor_move_relative()`。

### K230 解析函数

`k230.c/.h` 接收并解析以下目标跟踪帧：

```text
$<length>,15,<x>,<y>,<width>,<height>,#
```

其中 `x`、`y` 是目标框左上角坐标。`K230_ParseFrame()` 校验帧头、帧尾、长度、功能号和数值范围，并计算目标中心点；`K230_PollTarget()` 在主循环中取出 UART 中断接收的完整数据帧。代码还统计完整帧、有效帧、解析错误、溢出和丢帧数量。

### 闭环/PID 模板

`gimbal_control.c/.h` 提供双轴闭环控制模板，包括目标中心低通滤波、像素误差换算为角度、死区、单次修正限幅、轴向软限位和电机忙状态判断。

当前实际控制是 **P 控制模板**：Yaw/Pitch 分别使用 `GIMBAL_YAW_KP` 和 `GIMBAL_PITCH_KP`。`Ki`、`Kd`、积分限幅和微分滤波尚未加入，后续可在取得真实运行日志后继续扩展为完整 PID，避免在未标定硬件上直接使用不合适的参数。

### 尚未标定的摄像头与激光距离

当前未设定摄像头光心与激光光轴的实际距离：

```c
#define GIMBAL_LASER_CAMERA_DISTANCE_MM (0.0f)
#define GIMBAL_LASER_OFFSET_CALIBRATED  (0U)
```

因此现阶段视差补偿等效为关闭状态，不能据此承诺激光命中靶心。实物测试前需要测量光心间距、确认偏移方向，并校准相机焦距、两路电机轴映射和旋转正负方向。

## 主流程

`main.c` 的初始化顺序为：

1. `SYSCFG_DL_init()`：初始化芯片外设。
2. `step_motor_init()`：初始化电机 1、2。
3. `K230_Init()`：初始化 K230 UART 接收状态和中断。
4. `GimbalControl_Init()`：初始化闭环状态，并为两个电机设置速度。
5. 主循环调用 `K230_PollTarget()` 和 `GimbalControl_Update()`，根据目标中心误差下发非阻塞相对步数命令。

更完整的引脚、串口协议、标定项、诊断输出和已知风险见 `hardware_configuration.txt` 与 `CHANGE_20260714_01_K230_LASER_GIMBAL_TRACKING.txt`。

## 原始 TI 工程信息

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0G3507 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873).

## 编译与运行

使用 Code Composer Studio 导入工程后先 Refresh，再执行 Project → Clean 和 Build。新增模块为 `k230.c`、`gimbal_control.c`，编译日志中应能看到这两个源文件，否则生成的可能仍是旧版镜像。
