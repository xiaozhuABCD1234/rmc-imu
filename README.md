# RoboMaster 开发板 C 型 BMI088 IMU 串口上位机工程

适配 RoboMaster 开发板 C 型（STM32F407IGHx，板载 12 MHz HSE）。通过板载 SPI1
读取 BMI088 六轴 IMU 与温度，以固定 1 kHz 采样，用 VOFA+ 的 **JustFloat** 协议
经串口 DMA 发给上位机。

工程骨架（HAL / CMSIS / GCC 启动文件 / CMake 交叉编译）参考
`dual-mid360-rmc-time-sync`；引脚与寄存器配置以《RoboMaster 开发板 C 型用户手册》
和 BMI088 数据手册为准。

## 功能概览

| 功能 | 实现 |
| --- | --- |
| IMU 读取 | SPI1 主机，5.25 MHz，模式 0；加速度计 + 陀螺仪 + 温度 |
| 采样节拍 | TIM3 更新中断 1 kHz（帧率与主循环耗时无关） |
| 数据上报 | USART1（J30）2000000 8N1，DMA，JustFloat 10 通道，44 B/帧 |
| 恒温控制 | TIM10_CH1 (PF6) 驱动加热电阻，PI 闭环，目标 40 ℃，10 Hz |
| 状态指示 | 红灯=IMU 未就绪，绿灯=推流中，蓝灯=1 Hz 心跳 |
| 容错 | IWDG ~4 s；IMU 掉线每秒自动重初始化；读失败 1 s 关加热 |
| 其它 | 未用 IST8310、USB、FIFO、数据就绪中断 |

## 硬件接口

BMI088 焊接在板上，**采 IMU 不需要任何接线**；只有上报数据才需要串口。

| 信号 | STM32 | 说明 |
| --- | --- | --- |
| SPI1 SCK / MISO / MOSI | PB3 / PB4 / PA7 | AF5；PB3/PB4 同时是 JTDO/NJTRST，配成 AF5 即释放 JTAG，SWD 不受影响 |
| CS1_Accel / CS1_Gyro | PA4 / PB0 | 软件片选 |
| INT1_Accel / INT1_Gyro | PC4 / PC5 | 本工程不用 |
| Heat_Power | PF6 / TIM10_CH1 | 加热电阻，100 Hz PWM |
| 串口 TX / GND | J30 pin2 (PA9) / J30 pin3 | 3.3 V LVTTL，接 USB-TTL 的 RX / GND |

串口：**2000000 bps，8N1，无流控**。外壳丝印的 `UART1`/`UART2` 与 STM32 外设
编号**相反**（丝印 UART1 = USART6/J29，丝印 UART2 = USART1/J30），以针位为准。

> **加热需要 XT30 输入电源。** 加热丝的 `Heat_Power` 是 5 V，而 5 V 只来自
> XT30（8~28 V）或 micro-USB。只插 DAPLink 时 `VCC_3V3` 有电、MCU/IMU/串口
> 都能跑，但加热丝一点功率都没有（LED 也是灭的，因为 LED 阳极同接 5 V）。

## JustFloat 协议

```text
frame = [float32 ch0][float32 ch1] ... [float32 ch9][00 00 80 7F]
```

小端 IEEE-754 单精度；帧尾 `00 00 80 7F` 是 `0x7F800000`（+∞）的小端字节序，
VOFA+ 用它作为帧结束标志。本工程 10 通道，一帧 **44 字节**。
参考 <https://www.vofa.plus/docs/learning/dataengines/justfloat>

| 通道 | 含义 | 单位 | | 通道 | 含义 | 单位 |
| --- | --- | --- | --- | --- | --- | --- |
| 0/1/2 | 陀螺仪 X/Y/Z | deg/s | | 7 | 上电计时**整秒** | s |
| 3/4/5 | 加速度计 X/Y/Z | m/s² | | 8 | 上电计时**小数部分** | s (0~1) |
| 6 | IMU 温度 | ℃ | | 9 | 加热占空比 | 0~1 |

通道 7+8 合起来是上电计时，兼作**丢帧指示**：两者之和正常每帧步进 1/采样率，出现跳变
即丢了数据。

### 为什么时间要占两个通道

JustFloat 每通道就是一个 float32，而 float32 的精度随数值线性变差（尾数 24 位）。把
"上电以来经过了多少秒"塞进**一个**通道时，最小步长会随时间增长：

| 板子 uptime | 单通道 float32 秒的最小步长 |
| --- | --- |
| 10 min | 61 µs |
| 1 h | 244 µs |
| 2.3 h | 0.98 ms |
| 4.6 h | 1.95 ms |

到 2.3 小时就已经比 1 ms 的采样周期还粗了。拆成"整秒 + 秒的小数部分"之后：

- 整秒在 2²⁴ 秒（194 天）以内都是**精确**的整数（float32 可精确表示 ≤2²⁴ 的整数）；
- 小数部分始终落在 [0,1)，float32 在这个区间的分辨率恒为 **60 ns**。

所以时间基准不再随时间退化。实测每帧步进与 1 ms 的偏差不超过 **±47 ns**，4 s 累计漂移
**9 ns**（改前在 1 小时处是 ±122 µs）。

## 速率上限与推荐

| 瓶颈 | 上限 |
| --- | --- |
| BMI088 陀螺仪 ODR | **2000 Hz**（`GYRO_BANDWIDTH` 最高值） |
| BMI088 加速度计 ODR | 1600 Hz；2 kHz 采样时平均 1.25 倍重复 |
| 串口带宽 | 吞吐 / 44 字节：2 Mbps → 4500 Hz，921600 → 2100 Hz |
| SPI 轮询 | ~45 µs/次（3 次 HAL 传输共 19 字节），2 kHz 时占 CPU ~9 % |

**推荐 1 kHz**。同板同条件 A/B 实测（只改采样率与陀螺仪滤波）：

| 配置 | 陀螺仪静止 std [deg/s] | 加速度计 std [m/s²] |
| --- | --- | --- |
| **1 kHz + 116 Hz（默认）** | **0.204 / 0.288 / 0.248** | 0.031 / 0.029 / 0.029 |
| 2 kHz + 230 Hz | 0.505 / 0.604 / 0.660 | 0.035 / 0.033 / 0.032 |

2 kHz 的代价是陀螺仪噪声 **2.1~2.7 倍**，收益只有群延迟从 ~1.4 ms 降到 ~0.7 ms。
按用途选：

| 用途 | 建议 |
| --- | --- |
| 姿态解算 / EKF / 机器人控制 | **1 kHz**（配 116 Hz 滤波） |
| 只在上位机看波形 | 200~500 Hz 足够，曲线更好读 |
| 高频振动 / 冲击记录、亚毫秒延迟控制环 | 2 kHz，接受噪声代价 |
| 喂给上位机 SLAM / LIO | 1 kHz 偏多；时间同步与低抖动比采样率重要 |

要改只动两个宏，采样率与滤波器带宽必须配对：

| 采样率 | `IMU_TASK_RATE_HZ` | `BMI088_GYRO_BANDWIDTH_VALUE` | `BMI088_ACC_ODR_VALUE` |
| --- | --- | --- | --- |
| 1 kHz | 1000 | 0x02（1000 Hz / 116 Hz） | 0x0B（800 Hz） |
| 2 kHz | 2000 | 0x01（2000 Hz / 230 Hz） | 0x0C（1600 Hz） |

波特率不用跟着改（2 Mbps 在两种速率下只占 20~40%）。TIM3 的
`Period = (10000 / IMU_TASK_RATE_HZ) - 1`，所以速率要能整除 10000。

## 固件结构

```text
Core/Src
├── main.c              时钟、GPIO、SPI1、USART1、TIM3、TIM10、IWDG、状态灯
├── bmi088.c/.h         BMI088 驱动：初始化、量程/带宽配置、换算（换算系数由寄存器常量推导）
├── justfloat.c/.h      JustFloat 组帧 + UART DMA 发送队列（与外设无关）
├── imu_heater.c/.h     TIM10_CH1 加热电阻 PI 闭环
├── imu_task.c/.h       1 kHz 采样状态机、通道组帧、容错
├── stm32f4xx_hal_msp.c 各外设 MSP（引脚、时钟、DMA、NVIC）
└── stm32f4xx_it.c      只用到的四个中断向量
```

```text
TIM3 更新中断 (1 kHz) ──置标志──► 主循环 ImuTask_Poll
                                    ├─ SPI：陀螺仪 6B + 加速度计 6B + 温度 2B
                                    ├─ 换算 → 组帧 → JustFloat_Send 入队
                                    ├─ 每 100 个样本跑一次加热 PI
                                    └─ JustFloat_Poll → HAL_UART_Transmit_DMA
                                          ▲ USART1 TxCplt 回调清 in-flight
```

所有 SPI 访问都在主循环，中断只置一个计数标志：既保证固定帧率，又不在中断里
做阻塞式总线操作。

## 关键配置

| 配置项 | 位置 | 默认值 |
| --- | --- | --- |
| 波特率 | `main.h` `APP_UART_BAUD_RATE` | 2000000 |
| 采样率 | `imu_task.h` `IMU_TASK_RATE_HZ` | 1000 Hz |
| 通道数 | `justfloat.h` `JUSTFLOAT_CHANNEL_COUNT` | 10（须与 `imu_task.c` 一致） |
| 发送队列深度 | `justfloat.h` `JUSTFLOAT_QUEUE_DEPTH` | 4（同时在途 3 帧） |
| 加热目标 / 上限 | `imu_heater.h` | 40.0 ℃ / 0.85 |
| 加热开关 | `imu_heater.h` `IMU_HEATER_ENABLE` | 1 |
| 看门狗 | `main.h` `APP_ENABLE_IWDG` | 1 |
| 量程 | `bmi088.c` `BMI088_ACC_RANGE_VALUE` / `BMI088_GYRO_RANGE_VALUE` | ±6 g / ±2000 dps |

换算系数由量程寄存器常量推导（`s_accel_ms2_per_lsb`、`s_gyro_dps_per_lsb`），
不存在第二份量程数字会对不上：

```text
加速度 [m/s²]  = raw × 9.80665 × 量程g / 32768
角速度 [deg/s] = raw × 量程dps / 32768
温度 [℃]       = temp11 × 0.125 + 23     (temp11 = TEMP_MSB×8 + TEMP_LSB/32，11 位补码)
```

## BMI088 三个坑

### 1. 加速度计上电是 I2C 模式，且切换后第一次读无效

加速度计无视 PS 引脚，上电总是 I2C 模式，需要一个 CSB1 上升沿才切到 SPI；数据
手册 6.1 节同时说明：切换后那次 `ACC_CHIP_ID` 读**返回值就是无效的**。

只读一次并校验会拿到 `0xFF`（MISO 无人驱动的电平）→ `Bmi088_Init()` 失败、红灯
常亮、完全不推流，而同一时刻陀螺仪 `GYRO_CHIP_ID` 读回正确 `0x0F`，很容易误判
成接线或 SPI 模式问题。所以 `Bmi088_Init()` 里：CSB1 低脉冲 1 ms（微秒级不够）
→ 丢弃一次 dummy 读 → 软复位 → **再切一次**（软复位可能打回 I2C）→ 最多重试 3 次。

### 2. 加速度计读寄存器要多一个 dummy 字节

陀螺仪读：`地址|0x80` 后紧接数据。加速度计读：中间多一个内容不可预测的 dummy
字节（读 6 字节数据要收发 8 字节）。写操作之间需要 ≥2 µs 空闲（`tIDLE_wacc`）。
SPI 模式 0 和 3 都行；最大 10 MHz。

### 3. 加热的 5 V 只来自 XT30 输入

见上文「硬件接口」的提示。判据：温度平在室温且 `heater` 顶在 `0.85` 上限，就是
加热丝没功率（正常工作时应稳定在 40.000 ℃、占空比 ~0.16）。想确认输入电压可以
把 `ADC_BAT`（PF10 / ADC3_IN8，分压约 1/10）当一次性的万用表：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "init; halt" \
  -c "mww 0x40023844 0x00021410" -c "mww 0x40021400 0x00302000" \
  -c "mww 0x40012304 0x00010000" -c "mww 0x4001222C 0x00000000" \
  -c "mww 0x40012234 0x00000008" -c "mww 0x40012210 0x07000000" \
  -c "mww 0x40012208 0x00000001" -c "sleep 20" \
  -c "mww 0x40012208 0x57000001" -c "sleep 20" \
  -c "mdw 0x4001224C 1" -c "reset run; shutdown"
# VCC_BAT ≈ DR/4096 × 3.3 × 10.09   实测：只插 DAPLink ≈ 0.44 V，接 12 V 电池 ≈ 0.77 V
```

## 构建与烧录

交叉编译用 `arm-none-eabi`（GCC）。本机已装在
`~/.local/toolchains/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin`，
`build/` 已按该路径配置好：

```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

产物 `build/imu_justfloat.{elf,hex,bin}`。工具链不在 PATH 时用
`-DARM_TOOLCHAIN=<bin 目录>`；`-DCMAKE_BUILD_TYPE=Debug` 换 `-Og` 便于单步。

烧录用外部 DAPLink / CMSIS-DAP。本机 OpenOCD 装在
`~/.local/toolchains/xpack-openocd-0.12.0-7/bin`（免 sudo）。

若探针或串口报权限错误，跑一条一次性 udev 规则（`uaccess` 只授权当前登录用户，
不必重新插拔或重新登录）：

```bash
printf '%s\n' \
 'SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", ATTR{idProduct}=="0204", MODE="0660", GROUP="plugdev", TAG+="uaccess"' \
 'KERNEL=="hidraw*", ATTRS{idVendor}=="0d28", ATTRS{idProduct}=="0204", MODE="0660", GROUP="plugdev", TAG+="uaccess"' \
 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0660", GROUP="dialout", TAG+="uaccess"' \
 | sudo tee /etc/udev/rules.d/60-cmsis-dap.rules >/dev/null \
 && sudo udevadm control --reload-rules \
 && sudo udevadm trigger --subsystem-match=usb --subsystem-match=tty --subsystem-match=hidraw
```

```bash
cmake --build build --target flash          # program + verify + reset
```

其他调试器：`-DOPENOCD_INTERFACE=stlink`（或 `jlink`）。等价手写命令：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
        -c "program build/imu_justfloat.elf verify reset exit"
```

> 两个坑：`reset halt` 之后 openocd 的 `shutdown` **不会**让 CPU 恢复运行
> （表现是“烧完没数据”），诊断脚本末尾一定要跟 `reset run`；DAPLink 的 MSD
> 拖放（把 `.hex` 拷到 `/media/$USER/DAPLINK`）在本机这块 Interface 0253 上会
> 报 `Flash algorithm write verify command FAILURE`，而同时 openocd 完全正常。

## 上位机

### VOFA+

串口 → 端口 → 波特率 `2000000` → 8N1 无流控 → 数据引擎 **JustFloat** → 通道数
**9**。只想看部分曲线就在 VOFA+ 里隐藏通道，不用改固件。

### 用 Python 核对（可选）

Linux/macOS，只需标准库；Windows 用 pyserial 或直接开 VOFA+。

```bash
python3 - <<'PY'
import os, termios, time, struct, statistics
port, baud, CH = "/dev/ttyACM1", termios.B2000000, 10
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0
a[2] = termios.CREAD | termios.CLOCAL | termios.CS8; a[4] = a[5] = baud
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIFLUSH)
TAIL, SIZE = b"\x00\x00\x80\x7f", CH * 4 + 4
buf, frames, invalid, raw, t0 = bytearray(), [], 0, 0, time.monotonic()
while time.monotonic() - t0 < 3.0:
    try: c = os.read(fd, 16384)
    except BlockingIOError: time.sleep(0.002); continue
    raw += len(c); buf += c
    while True:
        t = buf.find(TAIL)
        if t < 0: break
        s = t + 4 - SIZE
        if s < 0: invalid += 1; del buf[:t + 4]; continue
        frames.append(struct.unpack("<%df" % CH, bytes(buf[s:s + CH * 4])))
        del buf[:t + 4]
os.close(fd); dt = time.monotonic() - t0
print("%d 字节, %d 帧, %.1f fps, 坏帧 %d" % (raw, len(frames), len(frames) / dt, invalid))
up = [f[7] + f[8] for f in frames]          # 整秒 + 小数
print("uptime 步进 %.6f ms" % (1000 * (up[-1] - up[0]) / (len(up) - 1)))
print("|accel| = %.3f m/s^2" % statistics.fmean(
    [(f[3] ** 2 + f[4] ** 2 + f[5] ** 2) ** 0.5 for f in frames]))
PY
```

## 实测结果

1 kHz @ 2 Mbps，接 XT30 电源：

| 项目 | 实测 |
| --- | --- |
| 帧率 / uptime 步进 | 999.0 fps / **1.0000 ms** |
| 丢字节 | **0**（抓到的字节数严格等于帧数 × 44） |
| \|加速度\| 静止 | 9.83 m/s²（未标定，真值 9.80665） |
| 陀螺仪静止噪声 | 0.20 / 0.28 / 0.24 deg/s |
| 温度 | 稳定 **40.000 ℃**，占空比 ~0.15 |
| LED | 绿灯常亮 + 蓝灯 1 Hz |

> 刚打开串口时可能丢掉半帧（解析器从头就是半帧），统计里体现为 1 个“坏帧”，
> 属正常重同步。真实指标是字节数与帧数严格成 40 倍、以及 uptime 步进。

## 排查清单

| 现象 | 检查 |
| --- | --- |
| 完全没有数据 | 用的是 J30（丝印 `UART2`）pin2/pin3，TX/RX 交叉，波特率 2000000 |
| 乱码 / 坏帧增长 | 波特率不匹配；数据引擎必须是 JustFloat，通道数 10 |
| 数据跳变 / 通道错位 | USB-TTL 在 2 Mbps 下丢字节，降到 921600 并降采样率复测 |
| 红灯常亮 | BMI088 通信失败：片选引脚定义、SPI1 是否被摄像头等占用 |
| 温度不升、`heater` 卡 0.85 | **先查 XT30 输入电源**（见「BMI088 三个坑」第 3 条），别先怀疑固件 |
| 烧完没数据 | openocd 停在 `reset halt` 后没跟 `reset run` |

## 已知限制

- 只做数据上报，不做姿态解算、零偏标定与温漂补偿；需要就在 `ImuTask_FillFrame()`
  之前插入自己的滤波/标定。
- 上报的是 BMI088 **芯片自身坐标系**的原始轴向数据，没做旋转；需要机体系就在
  `ImuTask_FillFrame()` 里做一次固定重映射。
- 采样率上限 2 kHz 是 BMI088 陀螺仪 ODR 上限，不是代码限制（见「速率上限」）。
- 未使用 FIFO、`INT1_Gyro` 数据就绪中断、IST8310 磁力计、USB CDC。
- USART1 只配了 TX，没有上位机下行通道。
