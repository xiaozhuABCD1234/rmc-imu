# rmc-imu — 开发板 C 型 BMI088 高精度 IMU 固件（STM32F407IGHx）

适配 RoboMaster 开发板 C 型。按**陀螺仪数据就绪中断**读取板载 BMI088，用 **84 MHz 硬件计数器**
给每个样本打时间戳，经 USART1 以**二进制帧（CRC 保护）**上报给上位机。

> 早期版本用 VOFA+ 的 JustFloat 文本协议，现已完全废弃：传输的是自定义二进制帧，
> 不再依赖 VOFA+。上位机 ROS 2 节点见 `~/Projects/rmc_imu_ws`。

## 为什么这样设计

早期版本用 TIM3 定时 1 kHz 去轮询传感器，有两个根本问题：

1. **传感器有自己的转换节拍**（BMI088 陀螺仪标称 ODR 1000 Hz，实测 DRDY 1001.3 Hz，
   与 MCU 晶振无关）。按定时器轮询时，读到的时刻与传感器真正转换的时刻最多差一整个
   传感器周期，而且约 1~5% 的读取会读到重复样本。
2. **时间戳的精度被两处吃掉**：采样时刻是"轮询节拍"而不是"转换时刻"；上报格式用
   float32 秒承载不断增大的绝对时间，1 小时后量化步长就涨到 244 µs。

现在：**PC5 上的陀螺仪 DRDY 中断就是采样时钟**，中断里读取 84 MHz 计数器作为该样本的
硬件时间戳。一帧 = 一次转换，不存在重复样本；时间戳精度只受中断入口延迟影响。

| | 定时器轮询（旧） | DRDY + 硬件时间戳（现在） |
|---|---|---|
| 采样触发 | TIM3 1 kHz 硬凑 | 陀螺仪 DRDY（实测 1001.3 Hz） |
| 样本新鲜度 | 约 1~5% 重复 | 每帧一次新转换 |
| 加速度计重复帧 | 21.6%（800 Hz ODR vs 1 kHz 轮询） | ~1.5%（1600 Hz ODR，即巧合下限） |
| 时间戳抖动 | ±122 µs（1 小时后）/采样时刻本身 ±0.5 ms 未知 | **±95 ns** |
| 测量通路 | float32 换算 | **int16 原始 LSB，全程整数** |

## 传输协议

固定 31 字节帧：

```
偏移  长度  字段
0     2     sync    0xAA 0x55
2     1     version 0x01
3     1     length  =25（payload 字节数）
4     25    payload
29    2     CRC-16/CCITT-FALSE（多项式 0x1021，初值 0xFFFF，覆盖字节 [2,29)）
```

payload：

```
偏移  长度  字段
0     8     uint64 timestamp_ns  上电以来纳秒，取自 DRDY 沿的 84 MHz 计数器
8     6     3 × int16 陀螺仪原始 LSB（X/Y/Z，小端）
14    6     3 × int16 加速度计原始 LSB（X/Y/Z，小端）
20    2     int16 温度码（11 位补码，0.125 ℃/LSB，23 ℃ @ 0）
22    1     uint8 GYRO_RANGE 寄存器值（0..4）
23    1     uint8 ACC_RANGE 寄存器值（0..3）
24    1     uint8 flags：bit0 板端跳样、bit1 板端 SPI 读失败
```

设计要点：

- **原始 int16 而不是换算后的浮点**：测量通路全程整数，且量程随帧上报，上位机自己算
  标度因数——单位只有一处定义，不会两边不一致。
- **固定长度 + 同步字 + CRC**：字节流自同步。上位机只要找同步字、校验长度和 CRC，任何
  丢字节、半帧、噪声都不会被误当成有效帧（旧 JustFloat 协议靠帧尾，需要额外猜"第一帧
  是否错位"）。
- 31 字节 × 1001 Hz ≈ 25 kB/s，在 2 Mbps 下只占 13%，余量充足。

标度因数（上位机用，也见 BMI088 数据手册 5.3.11 / 5.5.5）：

| GYRO_RANGE | 量程 | LSB/(°/s) | | ACC_RANGE | 量程 | LSB/g |
|---|---|---|---|---|---|---|
| 0 | ±2000 °/s | 16.384 | | 0 | ±3 g | 10920 |
| 1 | ±1000 °/s | 32.768 | | 1 | ±6 g | 5460 |
| 2 | ±500 °/s | 65.536 | | 2 | ±12 g | 2730 |
| 3 | ±250 °/s | 131.072 | | 3 | ±24 g | 1365 |
| 4 | ±125 °/s | 262.144 | | | | |

## 硬件接口

| 信号 | STM32 | 说明 |
|---|---|---|
| SPI1 SCK / MISO / MOSI | PB3 / PB4 / PA7 | AF5，5.25 MHz，模式 0 |
| CS1_Accel / CS1_Gyro | PA4 / PB0 | 软件片选 |
| **INT1_Gyro（陀螺仪 DRDY）** | **PC5** | EXTI5，**最高优先级中断**，全系统采样时钟 |
| INT1_Accel | PC4 | 未使用（加速度计随陀螺仪节拍一起读） |
| Heat_Power | PF6 / TIM10_CH1 | 加热电阻，100 Hz PWM，40 ℃ PI 闭环 |
| USART1 TX | J30 pin2 (PA9) | 2000000 8N1，DMA 发送；J30 pin3 = GND |

加热需要 XT30 输入电源（`Heat_Power` 是 5 V，只插 DAPLink 时加热丝没有功率）。诊断方法：
`imu/temperature` 停在室温且不升，就是加热丝没功率，不是固件问题。

## 时钟

| 时钟 | 值 | 用途 |
|---|---|---|
| SYSCLK | 168 MHz | HSE 12 MHz，PLLM 6 / PLLN 168 / PLLP 2 |
| APB1 / APB1 timer | 42 / **84 MHz** | TIM5 自由计数（时间戳） |
| APB2 / APB2 timer | 84 / 168 MHz | TIM10 加热 PWM；USART1（2 Mbps 分频零误差） |

时间戳换算：84 MHz → `ns = ticks × 1000 / 84`（截断误差 < 1 ns，64 位内 6 年不溢出）。
TIM5 是 32 位，每 51.15 s 溢出一次，由更新中断把高位补成 64 位。

## 关键配置

| 配置项 | 位置 | 默认值 |
|---|---|---|
| 波特率 | `main.h` `APP_UART_BAUD_RATE` | 2000000 |
| 陀螺仪量程 / 带宽 | `bmi088.h` `BMI088_GYRO_RANGE_VALUE` / `bmi088.c` | ±2000 dps / 0x02（ODR 1000 Hz，滤波 116 Hz） |
| 加速度计量程 / ODR | `bmi088.h` `BMI088_ACC_RANGE_VALUE` / `bmi088.c` | ±6 g / 0x0C（1600 Hz） |
| 加热目标 / 上限 | `imu_heater.h` | 40.0 ℃ / 0.85 |
| 加热控制周期 | `imu_task.h` `IMU_TASK_HEATER_PERIOD_MS` | 100 ms |
| 看门狗 | `main.h` `APP_ENABLE_IWDG` | 1（约 4 s） |

**加速度计 ODR 必须高于陀螺仪 DRDY 频率**（现在 1600 Hz > 1001 Hz），否则每次读到的
加速度计数据会重复。AB 实测：800 Hz ODR 时重复帧 21.6%，1600 Hz 时 1.5%（≈巧合下限）。

陀螺仪带宽决定噪声与延迟的取舍（同板同条件实测）：

| GYRO_BANDWIDTH | ODR / 滤波 | 静止 std |
|---|---|---|
| **0x02（默认）** | 1000 Hz / 116 Hz | **0.20 / 0.28 / 0.25 °/s** |
| 0x01 | 2000 Hz / 230 Hz | 0.51 / 0.60 / 0.66 °/s |

## 构建与烧录

```bash
export PATH="$HOME/.local/toolchains/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin:$HOME/.local/toolchains/xpack-openocd-0.12.0-7/bin:$PATH"
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --build build --target flash        # program + verify + reset
```

> `flash` 目标只在**配置时** openocd 在 PATH 里才会生成；否则配置阶段只给一条警告，
> 之后 `--target flash` 会报"没有规则可制作目标"。

串口权限一次性规则（`uaccess` 只授权当前登录用户）：

```bash
printf '%s\n' \
 'SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", ATTR{idProduct}=="0204", MODE="0660", GROUP="plugdev", TAG+="uaccess"' \
 'KERNEL=="hidraw*", ATTRS{idVendor}=="0d28", ATTRS{idProduct}=="0204", MODE="0660", GROUP="plugdev", TAG+="uaccess"' \
 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0660", GROUP="dialout", TAG+="uaccess"' \
 | sudo tee /etc/udev/rules.d/60-cmsis-dap.rules >/dev/null \
 && sudo udevadm control --reload-rules \
 && sudo udevadm trigger --subsystem-match=usb --subsystem-match=tty --subsystem-match=hidraw
```

## 不用上位机也能核对

独立解码器（只依赖标准库），可以交叉验证固件实现：

```bash
python3 - <<'PY'
import os, termios, time, struct
SYNC, FRAME, PAYLOAD = b"\xaa\x55", 31, 25
def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c
fd = os.open("/dev/ttyACM1", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd); a[0] = a[1] = a[3] = 0
a[2] = termios.CREAD | termios.CLOCAL | termios.CS8; a[4] = a[5] = termios.B2000000
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIFLUSH)
buf, frames, bad, t0 = bytearray(), [], 0, time.monotonic()
while time.monotonic() - t0 < 3.0:
    try: c = os.read(fd, 65536)
    except BlockingIOError: time.sleep(0.001); continue
    buf += c
    while len(buf) >= FRAME:
        i = buf.find(SYNC)
        if i < 0: del buf[:len(buf) - 1]; break
        if i > 0: del buf[:i]; continue
        f = bytes(buf[:FRAME])
        if f[3] != PAYLOAD or crc16(f[2:FRAME-2]) != struct.unpack_from("<H", f, FRAME-2)[0]:
            bad += 1; del buf[:1]; continue
        ts, = struct.unpack_from("<Q", f, 4)
        g = struct.unpack_from("<3h", f, 12); ac = struct.unpack_from("<3h", f, 18)
        tc, = struct.unpack_from("<h", f, 24)
        frames.append((ts, g, ac, tc, f[26], f[27], f[28])); del buf[:FRAME]
os.close(fd)
print(f"{len(frames)} 帧, 坏帧 {bad}, {len(frames)/(time.monotonic()-t0):.1f} fps")
d = [b[0] - a[0] for a, b in zip(frames, frames[1:])]
print(f"时间戳间隔中位 {sorted(d)[len(d)//2]/1e6:.6f} ms, "
      f"抖动峰值 {max(abs(x - sorted(d)[len(d)//2]) for x in d):.0f} ns")
print(f"陀螺仪 {[round(v/16.384, 3) for v in frames[-1][1]]} dps, "
      f"温度码 {frames[-1][3]} → {frames[-1][3]*0.125+23:.2f} C")
PY
```

## 实测结果

| 项目 | 实测 |
|---|---|
| 帧率 | **1001.3 Hz**（陀螺仪 DRDY 实测频率，非 1 kHz 整） |
| 帧长校验 | 155217 字节 = 5007 × 31 精确，**0 CRC 错 / 0 重同步字节** |
| 时间戳间隔 | 中位 998.67 µs，**与中位偏差峰值 95 ns**，严格递增 |
| 陀螺仪噪声 | 3.36 / 4.54 / 4.12 LSB = 0.205 / 0.277 / 0.251 °/s |
| 加速度计噪声 | 21.1 / 19.7 / 21.4 LSB ≈ 0.038 m/s² |
| \|加速度\| 静止 | 9.78 m/s²（未标定，真值 9.80665） |
| 温度 | 稳定 40.000 ℃ |
| 板端跳样 / 读失败 | **0 / 0** |
| 镜像 | `.text` 14596 B / `.bss` 2168 B |

## 已知限制

- **绝对时间精度受晶振限制**：板子时钟相对主机实测 **+49.9 ppm**（最小二乘拟合），即
  每小时约 180 ms。帧间间隔精度是 ±95 ns，但跨传感器绝对对齐需要在外部驯服时钟
  （PTP / 硬件触发 / 周期性重锚）；这不是改报文格式能解决的。
- 加速度计是未标定原始值：零偏与标度因数未补偿，静止模长 9.78 m/s²。
- 116 Hz 低通带来的群延迟约 1.4 ms（恒定偏移，融合时可标定补偿）。
- 陀螺仪 DRDY 实测 1001.3 Hz 是**传感器内部振荡器**决定的（约 +0.13%），与 MCU 晶振无关；
  上位机不能假设 1 ms 整。
- 未使用加速度计 DRDY（PC4）、IST8310 磁力计、FIFO。
- 上报的是 BMI088 芯片自身坐标系的轴向数据，未做机体系旋转。

## 踩过的坑

1. **加速度计上电是 I2C 模式**，需要一个 CSB1 上升沿切到 SPI，而切换后那次 `ACC_CHIP_ID`
   读返回无效值（数据手册 6.1）。只读一次会拿到 `0xFF`。做法：CSB1 低脉冲 1 ms → 丢弃
   一次 dummy 读 → 软复位 → 再切一次 → 最多重试 3 次。加速度计读寄存器还比陀螺仪多一个
   dummy 字节。
2. **外设时钟没开时，HAL 的配置寄存器写入会被静默丢弃**。`HAL_TIM_Base_Init()` 不检查
   时钟是否使能，`__HAL_RCC_TIM5_CLK_ENABLE()` 必须放在 `HAL_TIM_Base_MspInit()` 里
   （即 `HAL_TIM_Base_Init` 之前）。症状是"定时器永远不计数"，而且没有任何报错——本次
   开发中时间戳全 0 就是这个原因。
3. **`flash` 目标依赖配置时 PATH 里有 openocd**（见上文构建一节）。
4. DAPLink 的 MSD 拖放烧录在本机这块 Interface 0253 上会报
   `Flash algorithm write verify command FAILURE`，用 openocd 正常。
5. `reset halt` 之后 openocd 的 `shutdown` 不会让 CPU 恢复运行（表现为"烧完没数据"），
   诊断脚本末尾一定要跟 `reset run`。
