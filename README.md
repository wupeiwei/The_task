# 模拟 RM 步兵控制链路

基于两块 STM32F103C8T6 最小系统板，模拟 RoboMaster 步兵机器人的"云台板 + 底盘板"双板 CAN 通信控制链路：云台板采集摇杆指令，控制 360° 舵机（云台 yaw 轴），并通过 CAN 将底盘控制信息下发给底盘板；底盘板解析指令，驱动直流减速电机（底盘轮子）完成速度闭环。

## 硬件结构

| 模块 | 云台板 | 底盘板 |
|---|---|---|
| 主控 | STM32F103C8T6 | STM32F103C8T6 |
| CAN 通信 | TJA1050（PA11/PA12） | TJA1050（PA11/PA12） |
| 摇杆 | ADC1 CH0/CH1（PA0/PA1，DMA 采样） | - |
| 舵机 | TIM1_CH1（PA8）50Hz PWM | - |
| 电机 | - | TIM1_CH1（PA8）20kHz PWM + TB6612（PB12/PB13 方向，PA1 STBY） |
| 编码器 | - | TIM3 CH1/CH2（PA6/PA7，4 倍频 44 计数/圈） |
| OLED | - | I2C1（PB6/PB7），SSD1306 128x64 |
| 呼吸灯 | PC13（CAN 异常） | PC13（电机异常） |

## 软件架构（三层分层 + 双板单工程）

```
CMakeLists.txt      顶层工程：-DBOARD=gimbal|chassis 选择板型（同一工程+条件编译）
application/        应用层（预留）
components/         中间层：协议、CAN 收发、OLED 驱动（双板共用，条件编译区分）
boards/             硬件层：CubeMX 生成（chassis/ 与 gimbal/ 两个外设配置目录）
```

- 顶层 CMake 工程通过 `-DBOARD=` 选项（`cmake --preset gimbal` / `chassis`）选择板型，编译宏 `BOARD_GIMBAL` / `BOARD_CHASSIS` 区分板级逻辑——满足任务书"双板共用同一工程，通过条件编译区分"要求
- 两板外设配置分离（`boards/gimbal/`、`boards/chassis/` 各持 .ioc）：因外设资源真实冲突（TIM1 两板同为 PA8 但频率需求不同：舵机 50Hz vs 电机 20kHz；PA1 云台为 ADC 通道、底盘为 GPIO 输出），单 .ioc 无法表达两套外设
- 双板共用同一套 `components/` 源码，通过 `BOARD_GIMBAL` / `BOARD_CHASSIS` 编译宏区分板级差异（如 CAN 接收过滤器 ID、呼吸灯异常源）

## FreeRTOS 任务设计

| 任务 | 优先级 | 周期/触发 | 云台板 | 底盘板 | 职责 |
|---|---|---|---|---|---|
| motor_task | High | 10ms | - | ✅ | 编码器测速 + PID 闭环 + TB6612 输出 + 堵转/在线判定（500ms 计数域） |
| can_recv_task | AboveNormal | 队列阻塞 | ✅ | ✅ | CAN 帧解析 → 共享变量 |
| input_task | Normal | 10ms | ✅ | - | 摇杆采样/映射 + 舵机 PWM 输出 |
| can_send_task | Normal / BelowNormal | 5ms / 10ms | ✅ | ✅ | 周期打包发送协议帧 |
| health_task | Low | 20ms | ✅ | ✅ | 通信超时判定（失联清零目标） |
| led_task | Low | 1ms | ✅ | ✅ | 软件 PWM 呼吸灯 |
| display_task | Low | 100ms | - | ✅ | OLED 状态显示（含任务栈高水位） |

任务间数据共享采用 **volatile 共享变量**（控制类数据，只要最新值）+ **FreeRTOS 队列**（CAN 原始帧，每帧都要处理）。

## 板间通信方案（CAN 1Mbps）

- 标准帧，DLC=8，小端序，CRC-8（多项式 0x07）校验覆盖前 7 字节
- 云台 → 底盘：`0x101` 控制帧，5ms 周期
  - [0-1] 电机目标速度（±1000 RPM）、[2-3] 舵机目标转速、[4] 序号、[5] 状态字节、[6] 版本
- 底盘 → 云台：`0x201` 反馈帧，10ms 周期
  - [0-1] 指令回显、[2-3] 电机实际转速、[4] 序号、[5] 状态字节、[6] 版本
- 状态字节：控制帧位 0 舵机在线 / 位 1 板间通信；反馈帧位 0 电机在线 / 位 1 板间通信 / 位 2 电机异常
- 接收侧硬件过滤器只放行本板关心的帧，中断中仅搬数据（`xQueueSendFromISR`），解析放任务

## 核心算法

- **增量式 PID 速度环**（底盘电机）：`Δu = Kp(e−e₁) + Ki·e + Kd(e−2e₁+e₂)`，输出限幅 ±1000，参数待真机整定；失联时强制清零 PID 累积输出与历史误差，PWM 立即归零
- **M 法测速**：编码器计数差按真实时间窗换算 RPM（`diff × 60000 ÷ (44 × dt_ms)`，dt_ms 取 HAL_GetTick 差值，消除任务调度抖动；44 = 11 线 × 4 倍频）
- **堵转检测（计数域）**：500ms 窗口累计编码器计数，有指令且累计 < 3 计数（≈8 RPM）判堵转——规避 10ms 窗口 136 RPM/计数的低速量化盲区；电机在线同理（窗口内有指令且有响应）
- **摇杆映射**：12 位 ADC 减中点 2048 → 死区 ±50 滤抖动 → 线性映射 ±1000 RPM
- **软件 PWM 呼吸灯**：PC13 无硬件 PWM 通道，1ms 节拍 GPIO 翻转模拟 20 级亮度三角波
- **CRC-8 校验**：标准 CRC-8/ATM（poly 0x07），校验向量 123456789 = 0xF4，防总线干扰导致的脏数据

## 构建与烧录

```bash
# 在仓库根目录执行：同一工程按板型选择（任务书要求）
cmake --preset gimbal && cmake --build build/gimbal    # 云台板 → build/gimbal/gimbal_dual.elf
cmake --preset chassis && cmake --build build/chassis  # 底盘板 → build/chassis/gimbal_dual.elf
```

> 注意：`gimbal`/`chassis` preset 定义在**仓库根**的 `CMakePresets.json`；各板目录（`boards/<板>/`）内的 preset 为 CubeMX 生成的 `Debug`/`Release`，供单板调试用。

烧录使用各板目录下的 `openocd.cfg`（ST-LINK + OpenOCD），两板固件独立烧录。

## 测试与 CI

- `tests/test_protocol.py`：协议层测试（CRC 校验向量 0xF4、双帧回环、篡改拦截、56 种单比特错误），`python3 tests/test_protocol.py` 直接运行
- GitHub Actions（`.github/workflows/ci.yml`）：push/PR 自动编译两板 + 跑协议测试
- 运行时安全机制：栈溢出检测（`configCHECK_FOR_STACK_OVERFLOW=2` 栈顶标记法）、内存分配失败钩子、OLED 任务栈高水位显示（display_task 行 6 `STK` 字段）

## 开发环境与工具链

- **STM32CubeMX 6.17**：外设图形化配置，生成 CMake 工程（Toolchain 选 `CMake`）
- **CLion 2026.2**：编辑、编译、调试一体；使用内置 CMake + Ninja 生成器
- **ARM GCC 12.2.1**（arm-none-eabi）：交叉编译器，经各板目录下的 `cmake/gcc-arm-none-eabi.cmake` 工具链文件接入
- **OpenOCD 0.12.0 + ST-LINK**：烧录与调试，配置文件为各板目录下的 `openocd.cfg`

## 开发记录

每个功能模块独立 commit 并推送（见 Git 历史），便于回溯与评审。

## 改进空间

- PID 参数真机整定（当前 Kp/Ki/Kd 初值为 0）
- 舵机在线状态目前为 PWM 使能软标志，可加电流/堵转检测
- OLED 中文显示（需 16x16 字库）
