# 26 电控招新暑期任务

本仓库用于完成 26 电控招新暑期任务。任务要求已整理如下，原始任务书仅在本地保存，不纳入版本管理。

## 任务概览

任务一要求使用两块 STM32F103C8T6 模拟 RM 步兵机器人的云台板与底盘板控制链路：

- 云台端读取摇杆，控制 360 度舵机，并通过 CAN 向底盘端发送目标转速。
- 底盘端解析霍尔编码器反馈，以速度环闭环控制直流减速电机。
- 底盘端 OLED 显示舵机和电机的目标转速、实际转速、在线状态及板间通信状态。
- CAN 或电机异常时，对应板载 LED 显示呼吸灯效果。
- 必须使用 FreeRTOS；两块板共用同一工程，通过条件编译区分；代码按硬件层、中间层、应用层组织。

任务二要求理解乌龟步兵的云台、底盘、发射机构、板间通信、操控逻辑、上下位机通信，以及 PID、前馈 PID、滤波和斜坡算法，为答辩提问做准备。

## 开发环境与工具链

- **STM32CubeMX 6.17**：外设图形化配置，生成 CMake 工程（Toolchain 选 `CMake`）
- **CLion 2026.2**：编辑、编译、调试一体；使用内置 CMake + Ninja 生成器，工具链指向 Arm GNU Toolchain
- **ARM GCC 12.2.1**（arm-none-eabi）：交叉编译器，经各板目录下的 `cmake/gcc-arm-none-eabi.cmake` 工具链文件接入
- **OpenOCD 0.12.0 + ST-LINK**：烧录与调试，通过 CLion 的 "OpenOCD Download & Run" 运行配置调用，配置文件为各板目录下的 `openocd.cfg`（target 为 `stm32f1x.cfg`）

CLion 使用自带的 CMake 和 Ninja；ARM GCC 与 OpenOCD 需要能够被 CLion 的工具链环境找到。各板的 `CMakePresets.json` 已指定交叉编译工具链，使用 preset 时不需要重复传入工具链参数。

## 实现思路

### 工程分层

仓库采用双板单工程结构：硬件层由 CubeMX 按板分别生成，共享组件集中维护，应用层按公共、云台和底盘职责拆分：

```text
boards/
|-- gimbal/         # 云台板 CubeMX 工程（.ioc、Core、Drivers、cmake、openocd.cfg）
`-- chassis/        # 底盘板 CubeMX 工程（同上）
components/         # 中间层：PID、滤波、斜坡、CAN 协议、在线检测（两板共用）
application/
|-- common/         # 两板公共的应用接口与状态定义
|-- gimbal/         # 摇杆、舵机及云台通信任务
`-- chassis/        # 电机、编码器、OLED 及底盘通信任务
docs/               # 接线图、调参记录等文档
```

使用 `BOARD_GIMBAL` 和 `BOARD_CHASSIS` 编译宏选择板级初始化和应用任务。板型宏由 CMake 构建目标定义，条件编译限制在板级入口和少量差异点；公共协议和算法只保留一份。

首个阶段从 `boards/gimbal` 独立配置和编译。底盘工程落位后再增加顶层 CMake，每次配置只选择一块板并使用独立构建目录，避免两个 CubeMX 生成工程的内部目标重名。

### FreeRTOS 任务

建议先按职责划分，后续再根据实测负载调整周期和优先级：

| 任务 | 建议周期 | 主要职责 |
| --- | ---: | --- |
| `input_task` | 5-10 ms | ADC 采样、滤波、死区处理、摇杆到目标速度映射 |
| `motor_task` | 1-5 ms | 读取编码器速度、斜坡限幅、PID 计算、更新 TB6612 PWM |
| `can_task` | 事件驱动 | 从队列解析报文、发送控制量/状态和心跳 |
| `health_task` | 10-20 ms | 根据最后接收时间判断 CAN、电机和舵机链路状态 |
| `display_task` | 100 ms | 刷新 OLED，避免显示刷新阻塞控制环 |
| `led_task` | 10-20 ms | 正常状态指示与异常呼吸灯 |

CAN 接收中断只完成取帧、时间戳记录和入队，解析与业务处理放到任务中。周期任务使用 `vTaskDelayUntil` 保持稳定周期。

### 双板通信

在 `can_protocol` 中集中定义报文 ID、字段、单位、字节序和超时，不直接传输 C 结构体。一个最小方案包括：

- 云台到底盘：电机目标转速、舵机目标转速、序号和云台状态。
- 底盘到云台：电机目标/实际转速、电机在线状态、序号和底盘状态。
- 双向心跳或周期状态帧：用于板间在线检测，超时立即将输出置为安全值。

两端接入同一 CAN 总线并共地，只在总线两端放置 120 欧终端电阻。先用固定报文验证波特率、引脚映射和收发，再接入控制任务。

### 控制链路

1. 对摇杆 ADC 做中值/低通滤波、中心校准和死区处理，再映射成有符号目标速度。
2. 对目标速度增加斜坡限制，降低突然换向造成的电流冲击。
3. 根据编码器每个控制周期的计数差计算实际转速；换算必须包含编码器线数、倍频系数和减速比。
4. 固定周期执行速度 PID，输出限幅后转换成 TB6612 的方向和 PWM 占空比，并实现积分限幅或抗饱和。
5. 先完成开环方向和编码器极性检查，再闭环整定 PID，最后加入断联保护和 OLED。

普通三线 360 度舵机通常没有反馈，脉宽只能控制方向和速度，无法确认舵机本体是否真实在线。因此 OLED 中的“舵机在线”应明确表示云台控制/遥测报文未超时；若所发舵机带独立反馈线，再改为实际反馈判定。

## 实施阶段与提交建议

每完成一个可独立验证的阶段就提交，不要在最后一次性提交全部代码：

1. `chore: scaffold dual-board project skeleton with gimbal cubemx config`
2. `chore: add chassis cubemx base configuration`
3. `feat: add board selection and freertos task skeleton`
4. `feat: add can protocol and heartbeat`
5. `feat: add joystick input and servo control`
6. `feat: add encoder speed measurement`
7. `feat: close motor speed loop with pid`
8. `feat: add oled status page and fault indicators`
9. `docs: document design parameters and test results`

提交前执行 `git status` 和 `git diff --check`，确认没有构建产物、临时文件或密钥。推送到 GitHub 后将仓库设为 public，并检查网页上能看到完整的阶段性提交记录。

## 待记录参数

硬件到手并完成单模块测试后，在此补充可复现的配置：

- STM32CubeMX 和 HAL/FreeRTOS 版本（当前：CubeMX 6.17.0，FW_F1 V1.8.7）
- 两种板型的条件编译配置方式
- CAN 波特率、时序、报文表和超时时间
- 舵机 PWM 频率、中位脉宽和速度映射范围
- 编码器线数、倍频、减速比和转速计算公式
- 电机控制周期、PID 参数、输出限幅和斜坡参数
- 接线图、供电方式、公共地和终端电阻位置
