# DM-4310

基于 **STM32H723VGT6 + STM32 HAL** 的达妙 DM4310 电机控制项目，通过 DT7 遥控器控制正反转与转速，支持正弦变速“小陀螺”模式和 VOFA+ 实时波形监测。

工程采用裸机主循环与 TIM3 定时中断，不依赖 FreeRTOS。当前配置一台电机，使用 **MIT 模式中的速度给定**，不做位置保持。

## 功能

- UART5 + DMA 双缓冲 + 空闲中断接收 DT7 D-BUS 数据。
- 左摇杆上下控制速度，标称最大目标速度为 ±20 rad/s。
- 普通模式平滑加速、快速减速，遥控掉线后逐步降为零速度给定。
- 左拨杆下档进入单方向正弦变速模式，目标速度为 4～20 rad/s。
- FDCAN1 经典 CAN 通信，支持电机控制、寄存器读取、反馈解析与 bus-off 恢复。
- USART1 输出 8 路 VOFA+ 波形，监测速度、位置、力矩与角度。

## 硬件与接线

当前代码面向达妙 DM-MC-Board02 / DM-H7 底板与 STM32H723VGT6。更换板卡时需核对引脚和电源控制逻辑。

| 用途 | 外设 / 引脚 | 说明 |
| --- | --- | --- |
| 电机 CAN | FDCAN1，PD0 RX / PD1 TX | 经板载收发器连接电机 CAN H/L；经典 CAN，1 Mbps |
| 遥控接收 | UART5 RX，PD2 | 使用板上 D-BUS/SBUS 接口；解析器仅支持 DT7 D-BUS 18 字节帧 |
| 波形与日志 | USART1 TX PA9 / RX PA10 | 115200、8N1，接 USB-TTL 模块 |
| 24 V 输出 1 使能 | PC14 | 启动时拉高 |
| 24 V 输出 2 使能 | PC13 | 默认开启 |
| 对外 5 V 使能 | PC15 | 默认开启，给遥控接收机供电 |
| 烧录调试 | SWD | ST-Link 或 CMSIS-DAP |

UART5 配置为 100000 bps、8 位数据、偶校验、2 位停止位（HAL 中使用包含校验位的 9B 字长）。

查看波形时，将 USB-TTL 的 RX 接到 PA9、GND 接到板上 GND。板子独立供电时无需连接 USB-TTL 电源脚。当前固件没有 USB CDC，板载 USB-C 不能直接提供本工程的虚拟串口波形输出。

## 电机配置

当前值来自 `Core/Src/motor_control.c` 中的 `dm_motor_init()`：

| 参数 | 默认值 |
| --- | --- |
| CAN ID / ESC_ID | `0x01` |
| Master ID / 反馈帧 ID | `0x11` |
| 控制模式 | MIT（`mit_mode`） |
| CAN 波特率 | 1 Mbps |
| PMAX / VMAX / TMAX | 12.5 rad / 30 rad/s / 10 N·m |
| Kp / Kd | 0.0 / 0.5 |
| 力矩前馈 | 0.0 N·m |

使用达妙调试工具核对电机内部 ID、波特率和映射范围。**修改源码中的 ID 不会自动修改电机内部 ID。当前 Master ID 是 `0x11`，不是出厂默认的 `0x00`。**

启动时程序向指定电机写入控制模式，读取寄存器，然后失能、保存控制模式并重新使能。MIT 给定包含位置、速度、Kp、Kd 和力矩前馈；当前 Kp 为零，不参与位置控制，Kd 根据速度误差产生力矩。带负载时实际速度可能与给定存在偏差。

## 遥控操作

首次运行时将左摇杆回中、左拨杆置于上档或中档，再给系统上电。

| 操作 / 条件 | 行为 |
| --- | --- |
| 左拨杆上档或中档，左摇杆上下推动 | 根据摇杆方向和幅度给定正负速度 |
| 普通模式下摇杆回中 | 给定逐步降为零 |
| 左拨杆下档 | 进入小陀螺模式，忽略摇杆速度输入 |
| 左拨杆离开下档 | 返回摇杆控制 |
| 遥控未上线或掉线 | 目标归零，按减速斜率降低速度给定 |

遥控在线要求连续收到超过 5 帧，且最近一帧距当前不足 100 ms。掉线停车通过零速度给定实现，不直接切断电源或发送失能命令。

普通模式：

```text
目标速度 = 左摇杆竖直通道归一化值 × 20 rad/s
```

当前没有额外的摇杆中位死区处理，回中后若仍有微小速度给定，可检查原始通道值。

小陀螺模式：

```text
目标速度 = 12 + 8 × sin(相位) rad/s
频率 = 0.5 Hz，周期 = 2 s
进入模式时相位 = -π/2，初始给定 = 4 rad/s
```

该模式跳过普通加减速斜坡，因此切入时可能从上一给定直接跳到 4 rad/s。**小陀螺模式下摇杆回中不会停车，需要先将左拨杆拨离下档。**

## 常用参数

以下宏位于 `Core/Src/motor_control.c`：

| 宏 | 默认值 | 用途 |
| --- | --- | --- |
| `DM_RC_CONTROL_ENABLE` | `1` | 启用遥控；改为 0 后使用固定速度，不再执行遥控掉线停车分支 |
| `DM_RC_VEL_MAX` | `20.0f` | 摇杆满量程速度，rad/s |
| `DM_TARGET_VEL` | `5.0f` | 关闭遥控后的固定速度，rad/s |
| `DM_VEL_RAMP_UP` | `0.01f` | 每 1 ms 加速步长，等效 10 rad/s² |
| `DM_VEL_RAMP_DOWN` | `0.08f` | 每 1 ms 减速步长，等效 80 rad/s² |
| `DM_GYRO_ENABLE` | `1` | 启用小陀螺功能 |
| `DM_GYRO_SW` | `RC_SW_LEFT` | 小陀螺触发拨杆 |
| `DM_GYRO_BIAS` | `12.0f` | 正弦速度偏置，rad/s |
| `DM_GYRO_AMP` | `8.0f` | 正弦速度幅值，rad/s |
| `DM_GYRO_FREQ` | `0.5f` | 正弦频率，Hz |
| `DM_MIT_KP` / `DM_MIT_KD` | `0.0f` / `0.5f` | MIT 位置刚度与速度阻尼 |
| `DM_MIT_TOR_FF` | `0.0f` | 力矩前馈，N·m |

普通模式给定从 0 加速到 20 rad/s 约需 2 秒，从 20 降为 0 约需 0.25 秒，实际响应取决于负载。改变控制频率时需同步调整斜坡步长和正弦相位步长。

## VOFA+ 波形

默认选择 **USART1、115200、8N1、FireWater**，每 20 ms 输出一帧，约 50 Hz。配置位于 `application/vofa.h`。

| 通道 | 数据 | 单位 |
| --- | --- | --- |
| 0 | 实际下发速度给定（普通模式经过斜坡） | rad/s |
| 1 | 电机反馈速度 | rad/s |
| 2 | 电机反馈位置 | rad |
| 3 | 电机反馈力矩 | N·m |
| 4 | 左摇杆竖直通道归一化值 | 标称 −1～1 |
| 5 | 开机时间，整秒更新 | s |
| 6 | 反馈位置换算的有符号角度 | ° |
| 7 | 反馈角度对 360 取模 | °，范围 [0, 360) |

角度相对于电机已有零点，程序不自动设置零点，也不累计多圈转角。位置反馈回绕时角度可能跳变；未收到反馈时，零值不能证明电机处于零位。

可将 `VOFA_PROTOCOL` 改为 `VOFA_PROTOCOL_JUSTFLOAT`，同时调整上位机协议。默认 `VOFA_SILENCE_DM_LOG = 1`，波形初始化后关闭诊断文本。排查通信时可设为 0，并使用串口终端查看混合输出。

## 编译与烧录

### 命令行编译

需要 CMake 3.22 或以上、Ninja 和 GNU Arm Embedded 工具链。确保 `arm-none-eabi-gcc` 等程序在 PATH 中；工具链文件也会搜索其中列出的 STM32CubeCLT 安装目录。

在仓库根目录运行：

```sh
cmake --preset Release
cmake --build --preset Release
```

调试版本：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

产物为 `build/Release/DM.elf` 或 `build/Debug/DM.elf`，实际使用的链接脚本是 `STM32H723VGTx_FLASH.ld`。构建目录已被 Git 忽略。

需要 BIN 时手动转换：

```sh
arm-none-eabi-objcopy -O binary build/Release/DM.elf build/Release/DM.bin
```

### 烧录

使用 STM32CubeProgrammer 配合 ST-Link，选择 ELF 文件下载。`.vscode/launch.json` 提供 STM32Cube 扩展的 ST-Link 调试配置。

使用 CMSIS-DAP / DAPLink 和支持 STM32H7 的 OpenOCD 时：

```sh
openocd -f openocd_dap_h7.cfg -c "program build/Release/DM.elf verify reset exit"
```

该配置使用 SWD 和 `stm32h7x.cfg`，目标为 STM32H723。

## 项目结构

```text
Core/Inc/                    MCU 与电机驱动头文件
Core/Src/main.c              外设、电源初始化、电机启动与主循环
Core/Src/motor_control.c     电机配置、遥控映射、速度斜坡、小陀螺逻辑
Core/Src/dm_motor_ctrl.c     CAN 收发、MIT 打包、反馈解析与恢复
Core/Src/fdcan.c             FDCAN1 外设和引脚配置
Core/Src/usart.c             UART5 / USART1 配置
Core/Src/tim.c               TIM3 基础配置
application/remote_control.c DT7 解析与在线判断
application/bsp_rc.c         UART5 DMA 双缓冲接收
application/detect_task.c    接收时间记录，不是 RTOS 任务
application/vofa.c           波形采集与输出
Drivers/                    CMSIS 与 STM32H7 HAL
cmake/                      工具链及 CubeMX 构建配置
DM.ioc                      STM32CubeMX 工程
CMakePresets.json            Debug / Release 预设
openocd_dap_h7.cfg           CMSIS-DAP 烧录配置
```

执行流程：初始化外设和电源 → 设置 CAN 与电机参数 → 按实际时钟配置 TIM3 为 1 kHz → 设置并保存电机模式、使能 → 初始化遥控与波形输出。

TIM3 中断每 1 ms 调用 `motor_control()` 计算并发送给定；主循环处理 CAN 恢复、保活、日志与限频波形输出。

## 常见问题

| 现象 | 检查方向 |
| --- | --- |
| 电机不转 | 电机供电、CAN H/L、1 Mbps 配置、电机内部 ID，以及遥控在线状态 |
| 电机能转但反馈为零 | 电机 Master ID 是否与代码的 `0x11` 一致，反馈是否到达 |
| 遥控一直离线 | 接收机供电、对频、D-BUS 接口；当前不支持普通 25 字节 SBUS 帧 |
| 摇杆回中仍转 | 左拨杆是否处于小陀螺下档，以及中位偏差和力矩前馈 |
| VOFA+ 无有效波形 | USB-TTL 接线、115200 / FireWater；通道 5 可确认数据是否持续输出 |
| 看不到诊断日志 | 波形初始化后默认关闭 `dm_log`，检查 `VOFA_SILENCE_DM_LOG` |
| CAN 发送异常 / bus-off | 接线、终端、电源和波特率；自动恢复不能代替物理链路修复 |

CAN 默认固定使用 PD0/PD1，`DM_CAN_PROBE_ENABLE = 0`，不会自动切换到 PA11/PA12。普通状态灯默认关闭，不应依赖灯闪判断程序状态。

### CubeMX 重新生成注意事项

`DM.ioc` 与手工扩展代码不是完全等价的：TIM3 源文件由顶层 CMake 手动加入，控制频率在 `main.c` 中重新计算，遥控 DMA 双缓冲由应用层补充。重新生成后应检查 TIM3、UART5 DMA / 空闲中断、FDCAN 引脚及应用源文件列表。

部分历史注释仍提到其他 ID 或“速度模式”，实际行为应以赋值、宏定义和控制分支为准。

## 许可证

仓库暂未为自编代码声明统一许可证。CMSIS、STM32 HAL 等第三方文件保留各自的许可证与版权声明，使用或分发时请查看对应目录的许可文件。
