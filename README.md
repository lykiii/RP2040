# 基于树莓派 RP2040 的音高检测与音准训练器

> 2026 寒假在家一起练项目 ｜ 华中科技大学 ｜ 李奕恺

![封面](https://qn.eetree.cn/FvIofA0Tkcdm6C1IlDpNV8Ua7Qdo?imageView2/2/w/768/h/432)

`嵌入式系统` `MPU` `ADC` `FFT` `MicroPython` `ulab` `ST7789` `MMA7660`

本项目基于 RP2040 系统板（2M Flash、双核 Cortex-M0+、板载姿态传感器 MMA7660、ST7789 240×240 LCD），通过麦克风采集音频信号，经 FFT 频域分析估算基频，实现**音高检测**与**音准训练**两种模式，并支持姿态传感器手势快捷操作。

**电子森林项目主页**：<https://www.eetree.cn/project/4946>

---

## 目录

- [一、设计目标](#一设计目标)
- [二、准备工作](#二准备工作)
- [三、方案框图与设计思路](#三方案框图与设计思路)
- [四、实现过程](#四实现过程)
- [五、实物演示](#五实物演示)
- [六、后记](#六后记)

---

## 一、设计目标

1. **ADC 采样音频**并进行基础预处理，使用频谱（FFT）方法估算基频与置信度。
2. **LCD 显示**当前音高、目标音高与偏差指示条，误差达到阈值时给出蜂鸣器提示。
3. **音准训练模式**：随机给出目标音高，用户保持 2 秒内稳定即得分，记录最高分。
4. **姿态传感器快捷操作**：倾斜切换目标音高，摇一摇重新开始。

## 二、准备工作

### 1. 硬件连接

麦克风模块与 RP2040 的连接如下：

| 麦克风模块 | RP2040 |
|:---:|:---:|
| 3.3V | 3.3V |
| GND | GND |
| Aout | AIN0 (ADC0, GP26) |
| Ain | Mic（需短接 Ain 与 Mic） |

### 2. 开备环境

1. **Thonny**：安装参考 <https://class.eetree.cn/p/t_pc/goods_pc_detail/goods_detail/p_61cbd331e4b01af4136ea9d8?type=3>
2. **MicroPython 固件**：官网直接下载的 `.uf2` 不含信号处理函数，需引入 [ulab](https://github.com/v923z/micropython-ulab) 库，在 Linux 环境下结合官方 MicroPython 自行编译 `.uf2` 文件。按住 RP2040 板上的 BOOTSEL 键，连接电脑，将编译好的 `.uf2` 拖入出现的磁盘即可完成烧录。
3. **文档参考**：
   - MicroPython 官方文档：<https://docs.micropython.org/en/latest/index.html>
   - ulab 官方文档：<https://micropython-ulab.readthedocs.io/en/latest/>

### 3. 参考例程

电子森林 GitHub 开源代码：<https://github.com/EETree-git/RP2040_Game_Kit/tree/main/%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6%E4%BE%8B%E7%A8%8B>

### 4. 代码目录结构

| 文件 | 说明 |
|:---|:---|
| `main.py` | 主程序（状态机 + 音频采样 + FFT + UI） |
| `st7789.py` | ST7789 LCD 屏幕驱动 |
| `buzzer_music.py` | 蜂鸣器驱动与音调表 |
| `mma7660.py` | MMA7660 三轴姿态传感器驱动 |
| `fonts/vga1_16x32.py` | 大号字符字库 |
| `fonts/vga2_8x8.py` | 小号字符字库 |
| `test/` | 测试代码目录 |

### 5. 使用说明

1. 上电开机，编译下载程序，运行。
2. 由摇杆、按键等控制程序运行状态。

## 三、方案框图与设计思路

![方案框图](https://qn.eetree.cn/Fv5uU9LbC_YJkB7zV0K5Lr220NlS?attname=%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE+2026-03-19+202348.png)

项目主要难点在**音频采样与 FFT 算法**的使用。需要引入专门在资源受限的单片机上运行的数学库 ulab，与标准 MicroPython 编译生成 `.uf2` 文件后，才能在开发板上进行较为复杂的数学运算。

实现人机交互需要掌握对外设的控制，主要难点是**显示屏的使用**和**处理器与姿态传感器的数据互通**。其中，如何通过姿态传感器的数据实现对程序的控制——检测摇一摇和姿态倾斜——是关键。从以上问题出发进行代码编写，是完成本项目的着手点。

## 四、实现过程

### 1. 库导入

```python
import uos
from ulab import numpy as np
import test.st7789 as st7789
from test.fonts import vga2_8x8 as font1
from test.fonts import vga1_16x32 as font2
import random
import framebuf
from machine import Pin, SPI, ADC, PWM, I2C
import time, math, array
from utime import sleep_ms
import utime
from test.buzzer_music import music
from test.buzzer_music import tones
from test import mma7660
import struct
```

引入 ulab 数学库，引用电子森林开源驱动（`st7789.py`、`buzzer_music.py`、`mma7660.py`），还引入存储屏幕字符的 `vga2_8x8.py`、`vga1_16x32.py` 字库文件，引入随机函数 `random`，引入硬件驱动 `Pin`、`SPI`、`ADC`、`PWM`、`I2C` 等一系列必要的库函数。

### 2. 主要函数实现

#### 音高偏差条绘制

计算目标音与当前音高的偏差值（根据频率和音分的换算公式），限制最大范围后根据偏差设置方框条的**颜色**（绿色=准、红色=偏高、蓝色=偏低）、**长度**、**位置**。使用局部擦除旧方框的方式避免全屏清屏导致的闪烁。

```python
def update_tuner_bar(freq):
    global last_bar_x, last_bar_w

    if freq <= 0:
        return

    cents = 1200 * math.log(freq / benchmark_freq) / math.log(2)
    cents = max(-MAX_CENTS, min(MAX_CENTS, cents))
    px_per_cent = (display.width // 2) / MAX_CENTS
    bar_width = int(cents * px_per_cent)

    if abs(cents) <= 5:
        color = COLOR_PERFECT
    elif cents > 0:
        color = COLOR_SHARP
    else:
        color = COLOR_FLAT

    if last_bar_w > 0:
        display.fill_rect(last_bar_x, BAR_Y, last_bar_w, BAR_H, BG_COLOR)

    if bar_width > 0:
        new_x = CENTER_X
        new_w = bar_width
    elif bar_width < 0:
        new_x = CENTER_X + bar_width
        new_w = -bar_width
    else:
        new_x = CENTER_X
        new_w = 0

    if new_w > 0:
        display.fill_rect(new_x, BAR_Y, new_w, BAR_H, color)

    last_bar_x = new_x
    last_bar_w = new_w
    display.vline(CENTER_X, BAR_Y - 10, BAR_H + 20, BASE_LINE_COLOR)
```

#### 频率转音高

将测得的频率与字典里各个音高对应的频率进行对比，找出最接近的音高，返回标准音的符号、频率、与实测频率相差的音分值。

```python
def freq_to_pitch(measured_f):
    if measured_f <= 0:
        return "None", 0, 0

    closest_note = "C4"
    min_diff = float('inf')

    for note, std_f in PRACTICE_MODE_NOTE_FREQ.items():
        diff = abs(measured_f - std_f)
        if diff < min_diff:
            min_diff = diff
            closest_note = note

    target_std_f = PRACTICE_MODE_NOTE_FREQ[closest_note]
    cents = 1200 * (math.log(measured_f / target_std_f) / math.log(2))

    return closest_note, target_std_f, cents
```

#### ADC 采样与汉宁窗

官方库没有连续 ADC 采样函数，手动实现：设置目标采样率，配合延时函数，测量实际采样率后调整参数。使用 ulab 的线性空间存储采样数据，手动实现**汉宁窗**（Hanning Window）抑制频谱泄露，消除截断处的突变，提高弱信号检测能力。

```python
sound_adc = ADC(Pin(26))

SAMPLES = 512
SAMPLE_RATE = 8000
ACTUAL_SAMPLE_RATE = 4267
interval_us = int(1000000 / SAMPLE_RATE)

def collect_samples(n):
    raw_data = np.zeros(n, dtype=np.uint16)
    start = utime.ticks_us()
    for i in range(n):
        raw_data[i] = sound_adc.read_u16()
        utime.sleep_us(interval_us)

    end = utime.ticks_us()
    print("实际平均采样率:", 1000000 / ((end - start) / n))
    return raw_data

n = np.linspace(0, SAMPLES - 1, num=SAMPLES)
window = 0.5 - 0.5 * np.cos(2 * 3.14159265 * n / (SAMPLES - 1))
```

#### 按键中断与消抖

配置按键对应的 GPIO 口和中断回调函数。中断函数中只对标志位赋值，不进行复杂操作，避免影响主循环运行。所有标志位使用 `global` 关键字声明，并添加 200ms 消抖防止多次触发。

```python
buttonA = Pin(6, Pin.IN, Pin.PULL_UP)
buttonB = Pin(5, Pin.IN, Pin.PULL_UP)
buttonStart = Pin(7, Pin.IN, Pin.PULL_UP)
buttonSelect = Pin(8, Pin.IN, Pin.PULL_UP)

def button_handler(pin):
    global last_interrupt_time0, StartA, state, score, Select, init_menu_sign, StartB
    current_time0 = utime.ticks_ms()

    if utime.ticks_diff(current_time0, last_interrupt_time0) < 200:
        return
    last_interrupt_time0 = current_time0

    if pin == buttonA:
        if state == success:
            StartA = True
    elif pin == buttonB:
        if state == practice:
            score += 1
        if state == success:
            StartB = True
    elif pin == buttonSelect:
        if state == menu:
            Select = True
    elif pin == buttonStart:
        state = menu
        init_menu_sign = True

buttonA.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonB.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonStart.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonSelect.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
```

#### 姿态检测：倾斜切换

上电时记录开发板的初始姿态，主循环中持续检测。若姿态偏差大于阈值（tilt_threshold ≈ 25，接近 90° 翻转）则切换目标音高。包含回正逻辑防止重复触发，随机音切换时将字典转换为列表再调用 `random.choice`。

```python
def calibrate_tilt():
    global initial_x, initial_y, initial_z
    data = i2c.readfrom_mem(0x4C, 0x00, 3)
    def conv(val):
        val &= 0x3F
        if val & 0x20: val -= 64
        return val
    initial_x = conv(data[0])
    initial_y = conv(data[1])
    initial_z = conv(data[2])
    print(f"初始姿态: X={initial_x}, Y={initial_y}，Z={initial_z}")

def check_tilt_change():
    global initial_x, initial_y, is_tilted, benchmark_note, benchmark_freq
    data = i2c.readfrom_mem(0x4C, 0x00, 3)
    def conv(val):
        val &= 0x3F
        if val & 0x20: val -= 64
        return val
    curr_x = conv(data[0])
    curr_y = conv(data[1])
    diff_x = abs(curr_x - initial_x)
    diff_y = abs(curr_y - initial_y)
    if diff_x > tilt_threshold or diff_y > tilt_threshold:
        if not is_tilted:
            is_tilted = True
            notes_list = list(PRACTICE_MODE_NOTE_FREQ.keys())
            benchmark_note = random.choice(notes_list)
            benchmark_freq = PRACTICE_MODE_NOTE_FREQ[benchmark_note]
            play_target_hint(benchmark_note)
            return True
    else:
        if is_tilted:
            print("设备已回正")
        is_tilted = False
    return False
```

#### 姿态检测：摇一摇重启

当摇动发生时，三轴加速度剧烈变化，根据 |x|+|y|+|z| 之和判断。实测阈值 45 较为灵敏又不会误触发，包含 1 秒防重复触发逻辑。

```python
def check_shake():
    global last_shake_time, state, score, StartA
    current_time = utime.ticks_ms()
    if utime.ticks_diff(current_time, last_shake_time) < 1000:
        return False
    data = i2c.readfrom_mem(0x4C, 0x00, 3)
    def conv(val):
        val &= 0x3F
        if val & 0x20: val -= 64
        return val
    x = conv(data[0])
    y = conv(data[1])
    z = conv(data[2])
    shake_force = abs(x) + abs(y) + abs(z)
    if shake_force > 45:
        last_shake_time = current_time
        score = 0
        state = practice
        notes_list = list(PRACTICE_MODE_NOTE_FREQ.keys())
        benchmark_note = random.choice(notes_list)
        benchmark_freq = PRACTICE_MODE_NOTE_FREQ[benchmark_note]
        return True
    return False
```

#### 蜂鸣器提示音

切换目标音后，程序操作蜂鸣器发出对应频率的提示音。频率通过查 `tones` 字典获得。

```python
def play_target_hint(note_name):
    try:
        if note_name in tones:
            freq = tones[note_name]
            buzzer.freq(freq)
            buzzer.duty_u16(32768)
            utime.sleep_ms(300)
            buzzer.duty_u16(0)
    except Exception as e:
        print("播放提示音失败:", e)
```

### 3. 主循环状态机

主循环包含四个状态：**菜单（menu）**、**练习（practice）**、**成功（success）**、**测量（measure）**。

![状态机框图](https://qn.eetree.cn/Fk_m9azTOapyVFKaZLJVvUnVqZN-?attname=rp2040%E7%B3%BB%E7%BB%9F%E6%A1%86%E5%9B%BE.png)

**流程说明**：

- **Menu**：摇杆上下选择模式，Select 确认进入 Practice 或 Measure。
- **Practice**：ADC 采样 → 加窗 → FFT → 取频谱幅度最大索引 → 计算主频 → 频率转音高 → 绘制偏差条。音高与目标音匹配后开始计时，稳定 2 秒则计分，计满 6 分切换到 Success。支持倾斜切换目标音、摇一摇重启。
- **Success**：显示 "YOU WIN"。按 A 键保持分数继续，按 B 键清零重新开始。
- **Measure**：与 Practice 相比去掉计分和目标音，屏幕显示实时频率值。音频信号处理流程一致。

控制逻辑中使用大量标志位，方便代码逻辑关联，增强可读性，避免复杂嵌套。

```python
while True:
    if init_menu_sign == True:
        init_menu()
        init_menu_sign = False
        continue_sign = False

    if state == menu:
        display.text(font2, "MAIN MENU", 60, 40, st7789.YELLOW)
        x_val = xAxis.read_u16()
        if x_val > UP_THRESHOLD and not joy_moved:
            MODE_select_button = 1
            joy_moved = True
        elif x_val < DOWN_THRESHOLD and not joy_moved:
            MODE_select_button = 0
            joy_moved = True
        elif x_val > 25000 and x_val < 40000:
            joy_moved = False
        # ... 绘制菜单 UI + 模式选择逻辑 ...

    elif state == practice:
        check_tilt_change()
        check_shake()
        raw_samples = collect_samples(SAMPLES)
        signal = np.array(raw_samples, dtype=np.float)
        signal -= np.mean(signal)
        signal *= window
        fft_result = np.fft.fft(signal)
        magnitude = np.sqrt(fft_result.real**2 + fft_result.imag**2)
        max_index = np.argmax(magnitude[1:SAMPLES//2]) + 1
        estimated_freq = max_index * ACTUAL_SAMPLE_RATE / SAMPLES
        # ... 音高匹配 + 计分 + 绘制 UI ...

    elif state == success:
        display.fill(BG_COLOR)
        display.text(font2, "YOU WIN", CENTER_X - 55, BAR_Y, st7789.WHITE)
        # ... 按键继续/重启逻辑 ...

    elif state == measure:
        # ... 采样 + FFT + 显示频率（无计分）...
```

## 五、实物演示

四个状态的界面分别如下：

| 菜单界面 | 练习模式 |
|:---:|:---:|
| ![菜单](https://qn.eetree.cn/FrzqYc9J1xCUOBEg3hNIpfC-XA1m?attname=1.jpeg) | ![练习](https://qn.eetree.cn/FsLCbEZUis1ytrSRoJdkH6HTy0hq?attname=2.jpeg) |

| 成功界面 | 测量模式 |
|:---:|:---:|
| ![成功](https://qn.eetree.cn/FtTyNIq7Z796aVfhDUpHI9EM8wTT?attname=3.png) | ![测量](https://qn.eetree.cn/FkliDRaIXsLxta_xTZ84cHrsCWPX?attname=4.jpeg) |

## 六、后记

这次寒假按学校课程要求参加电子森林寒假一起练的活动，原以为比较简单，所以过年期间一直没做，等到年后发现时间比较紧，项目内容比较粗糙，还有许多可以改进的地方，如优化采样代码提高采样率、增加更多练习难度级别、添加音阶练习模式、改进低频检测算法等。

在完成本次项目时遇到了一些问题：固件烧录后死机、板载 Flash 有限不能适应高精度音频采样。本项目采用了**状态机编程**的思想同时运用了**中断技术**，巩固了编程能力，加强了信息检索利用能力。

感谢电子森林举办本次活动，通过这次活动收获颇丰。

---

**电路图**：

![电路图](https://qn.eetree.cn/FnJF8vI6wkI0S9gPp2nyhZM0iBFK)

**附件**：`test.zip`（工程源代码）、`firmware.uf2`（自编译含 ulab 固件）可在[电子森林项目页](https://www.eetree.cn/project/4946)下载。
