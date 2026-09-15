一、设计目标
1.ADC 采样音频并进行基础预处理，使用频谱或自相关方法估算基频与置信度。  
2.在 LCD 上显示当前音高、目标音高与偏差指示条，误差达到阈值时给出蜂鸣器提示。  
3.提供音准训练模式：随机给出目标音高，用户保持 2 秒内稳定即得分，记录最高分。  
4.使用姿态传感器实现快捷操作（例如倾斜切换目标音高或摇一摇重新开始）。  
二、准备工作
1.硬件连接
两模块连接如下:
麦克风模块
RP2040
3.3v   
3.3v
GND  
GND
Aout 
AIN0(ADC0)
Ain，Mic

注：要将麦克风模块的Ain和Mic连接起来
2.开发环境
（1）thonny。安装具体过程可参考https://class.eetree.cn/p/t_pc/goods_pc_detail/goods_detail/p_61cbd331e4b01af4136ea9d8?type=3
（2）micropython固件，因为官网直接下载的.uf2不含信号处理函数，故引入ulab库，在linux环境下，结合官方micropython自己编译.uf2文件，具体方法参照https://github.com/v923z/micropython-ulab，按住RP2040板子上的B键，连接电脑，可以发现电脑多出一个磁盘，将编译好的.uf2文件拖入该磁盘，磁盘消失，配置完成
（3）如何使用micropython我参考的是官方英文文档：https://docs.micropython.org/en/latest/index.html。如何使用ulab我参考的是其官方文档：https://micropython-ulab.readthedocs.io/en/latest/
3.参考例程
电子森林github开源代码：https://github.com/EETree-git/RP2040_Game_Kit/tree/main/%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6%E4%BE%8B%E7%A8%8B
4.代码目录结构
vga1_16x32.py 储存大号字符
vga2_8x8.py 储存小号字符
buzzer_music.py 蜂鸣器驱动模块
mma7660.py  姿态传感器驱动模块
st7789.py 屏幕驱动模块
main.py 主函数
5.使用说明
（1）上电开机，编译下载程序，运行
（2）由摇杆，按键等控制程序运行状态
三、方案框图与设计思路

      项目主要难点在音频采样与FFT算法的使用，我们需要引入专门在资源受限的单片机上运行的数学库ulab，如何与标准micropython编译生成.uf2文件参考上面链接，这样我们可以在开发板上进行较为复杂的数学运算。实现人机交互需要掌握对外设的控制，主要难点是显示屏的使用和处理器与姿态传感器的数据互通，其中如何通过姿态传感器的数据实现对程序的控制是检测摇一摇和姿态倾斜的关键。从以上问题出发进行代码编写，是完成本项目的着手点。

四、实现过程
main函数主要引用：
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
     引入ulab数学库，引用电子森林开源驱动，st7789.py，buzzer_music.py, mma7660.py,还引入存储屏幕字符的vga2_8x8.py, vga1_16x32.py文件，引入随机函数random，引入硬件驱动Pin, SPI, ADC, PWM, I2C等一系列必要的库函数，方便后续使用。
2.主要函数的实现过程
   以下是绘图音高偏差条的函数，注意引入全局变量才可以操作该变量，首先我们计算目标音与当前音高的偏差值（根据频率和音分的换算公式），并且限制一下最大范围，我们根据偏差设置方框条的颜色、长度、位置等，调用绘图函数时注意官方给的函数说明，如果音高没有变，我们可以保持而不用再次画图。
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
    以下函数是为了转换频率与音高，将测得的频率与字典里各个音高对应得频率进行对比，找出最接近的音高，并返回这个标准音的符号、频率、与给出频率得音相差得音分，主要是字典语法和数学函数的使用。
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
  官方库没有连续ADC采样函数，只有手动实现，设置目标采样率，设置相应的延时函数，测量实际采样率，然后调整参数，我们用一个线性空间来存储采样数据，window是我手动实现的汉宁窗，目的是抑制频谱泄露，消除截断处的突变，提高弱信号的检测能力，其本质是一个余弦函数。
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
    按键配置较为常规，我们配置按键对应的GPIO口和中断回调函数即可，在中断函数里，我们不进行复杂的操作，只对一些标志位进行赋值，这样不会影响程序的运行，注意操作的标志位是全局变量，要使用global关键字声明，同时添加消抖，防止多次触发。
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
        print(pin)
        if state == success:
            StartA = True
    elif pin == buttonB:
        print(pin)
        if state == practice:
            score += 1
        if state == success:
            StartB = True
    elif pin == buttonSelect:
        print(pin)
        if state == menu:
            Select = True
    elif pin == buttonStart:
        print(pin)
        state = menu
        init_menu_sign = True

buttonA.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonB.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonStart.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonSelect.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
      这两个是姿态检测函数，通过第一个函数在开发板一上电的时候，记录开发板的初始姿态，然后第二个函数会在主循环里循环检测，若姿态偏差大于我的判定条件则返回True，否则返回False。当我们检测到姿态转换时，我们可以执行目标音切换的逻辑，这个函数还涉及回正的逻辑，防止重复触发，随机音的切换是需要把字典转换为列表，再调用random函数。
def calibrate_tilt():
    global initial_x, initial_y, initial_z
    data = i2c.readfrom_mem(0x4C, 0x00, 3)
    def conv(val):
        val &= 0x3F
        if val & 0x20:
            val -= 64
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
        if val & 0x20:
            val -= 64
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
            print(f"检测到倾斜 90 度，新目标: {benchmark_note}")
            return True
    else:
        if is_tilted:
            print("设备已回正")
        is_tilted = False

    return False
       下面这个函数实现检测摇一摇，当此动作发生时，姿态的xyz坐标加速度会剧烈变化，我们可以根据三者绝对值之和进行判断，实际判断基准值是根据实际响应灵敏度调节的，发现设置45较为灵敏又不会误触发，此函数依旧设置防止重复触发的逻辑。
def check_shake():
    global last_shake_time, state, score, StartA
    current_time = utime.ticks_ms()
    if utime.ticks_diff(current_time, last_shake_time) < 1000:
        return False
    data = i2c.readfrom_mem(0x4C, 0x00, 3)
    def conv(val):
        val &= 0x3F
        if val & 0x20:
            val -= 64
        return val
    x = conv(data[0])
    y = conv(data[1])
    z = conv(data[2])
    shake_force = abs(x) + abs(y) + abs(z)
    if shake_force > 45:
        last_shake_time = current_time
        print("检测到摇一摇！重新开始...")
        score = 0
        state = practice
        notes_list = list(PRACTICE_MODE_NOTE_FREQ.keys())
        benchmark_note = random.choice(notes_list)
        benchmark_freq = PRACTICE_MODE_NOTE_FREQ[benchmark_note]
        return True
    return False
    该函数为提示音播放函数，如果切换了目标音那么程序会操作蜂鸣器发出相应频率的音，以提示用户。这个音的频率我们通过查字典来获得，直接调用蜂鸣器函数即可。
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
主循环有四个状态机，菜单、练习、检测和成功，循环逻辑如下：

      在menu下，有摇杆检测，读取摇杆的ADC值，执行判断逻辑，设置相应的标志位，后面是绘图部分，然后在practice模式下，我们首先进行了ADC采样和加窗操作然后进行具体的时域到频域的转换，也就是FFT算法的部分，我们先得到离散的频谱，这个函数返回的只是列表，我们先计算比较得到能量即幅度最大的列表索引值，然后根据官方指导文档里的公式，由索引计算出实际的频率，这样我们就得到了主频，然后分别执行我们上面提到的函数和官方的绘图函数，将我们想要的数据通过图形化的界面显示在屏幕上，后面音高检测后与目标音对比，若在范围之内，置标志位并记录时间，下次检测还符合则继续计时，若不符合则计时清零，这样只有当一个音在2秒内稳定符合要求时才会计分，然后是计分逻辑，当计分到达六分时会切换到success状态，但是注意，有一个continue_sign的标志位，在成功状态下检测按键标志，按A键会保持继续，返回practice状态，最后是measure状态，与practice模式相比去掉了计分和目标音部分，然后屏幕上显示的是频率而不是音高，音频信号处理与后者完全一致，在以上的控制逻辑中，使用了大量的标志位，代码中的一些标志在上面没有提及，使用标志位的好处是方便代码逻辑之间的关联，同时可以增强代码的可读性，避免了复杂的嵌套逻辑。
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


        color1 = st7789.BLUE if MODE_select_button == 0 else st7789.WHITE
        display.rect(40, 80, 160, 40, color1)
        display.text(font1, "PRACTICE MODE", 55, 92, color1)


        color2 = st7789.BLUE if MODE_select_button == 1 else st7789.WHITE
        display.rect(40, 140, 160, 40, color2)
        display.text(font1, "MEASURE MODE", 60, 152, color2)


        pointer_y = 152 if MODE_select_button == 0 else 92
        display.fill_rect(25, pointer_y, 10, 10, st7789.BLACK)
        pointer_y = 92 if MODE_select_button == 0 else 152
        display.fill_rect(25, pointer_y, 10, 10, st7789.CYAN)


        if MODE_select_button == 1:
            if Select == True:
                state = measure
                init_measure()
                Select = False
        elif MODE_select_button == 0:
            if Select == True:
                state = practice
                score = 0
                init_practice()
                Select = False


    elif state == practice:
        check_tilt_change()
        print(check_shake())
        raw_samples = collect_samples(SAMPLES)


        signal = np.array(raw_samples, dtype=np.float)
        signal -= np.mean(signal)
        signal *= window


        fft_result = np.fft.fft(signal)
        magnitude = np.sqrt(fft_result.real**2 + fft_result.imag**2)
        max_index = np.argmax(magnitude[1:SAMPLES//2]) + 1
        estimated_freq = max_index * ACTUAL_SAMPLE_RATE / SAMPLES
        print("主频约为: {} Hz".format(estimated_freq))
        closest_note, target_std_f, cents = freq_to_pitch(estimated_freq)
        display.text(font2, closest_note, CENTER_X - 10, BAR_Y - 50, st7789.WHITE)
        update_tuner_bar(estimated_freq)
        text0 = "score = {}".format(score)
        text2 = "highest_score = {}".format(highest_score)
        display.text(font2, "PRECTICE MODE", CENTER_X - 100, BAR_Y - 100, st7789.WHITE)
        display.text(font1, text2, CENTER_X - 100, BAR_Y - 60, st7789.WHITE)
        display.text(font1, text0, CENTER_X - 100, BAR_Y - 40, st7789.WHITE)
        display.text(font2, benchmark_note, CENTER_X - 10, BAR_Y + 50, st7789.WHITE)


        if closest_note == benchmark_note:
            if match_start_time == 0:
                match_start_time = utime.ticks_ms()
                print("音准正确，保持住...")


            duration = utime.ticks_diff(utime.ticks_ms(), match_start_time)


            if duration >= 2000 and not is_scored_for_this_note:
                score += 1
                is_scored_for_this_note = True


                notes_list = list(PRACTICE_MODE_NOTE_FREQ.keys())
                benchmark_note = random.choice(notes_list)
                benchmark_freq = PRACTICE_MODE_NOTE_FREQ[benchmark_note]


                play_target_hint(benchmark_note)
                match_start_time = 0
                is_scored_for_this_note = False
        else:
            if match_start_time != 0:
                print("音准断了，重新开始计时")
                match_start_time = 0


        highest_score = max(score, highest_score)


        if score > 5 and continue_sign == False:
            state = success


    elif state == success:
        display.fill(BG_COLOR)
        display.text(font2, "YOU WIN", CENTER_X - 55, BAR_Y, st7789.WHITE)
        if StartA == True:
            continue_sign = True
            state = practice
            score = score
            StartA = False
            init_menu()
        if StartB == True:
            state = practice
            score = 0
            StartB = False
            init_menu()


    elif state == measure:
        display.text(font2, "MEASURE MODE", CENTER_X - 100, BAR_Y - 100, st7789.WHITE)


        raw_samples = collect_samples(SAMPLES)
        signal = np.array(raw_samples, dtype=np.float)
        signal -= np.mean(signal)
        signal *= window


        fft_result = np.fft.fft(signal)
        magnitude = np.sqrt(fft_result.real**2 + fft_result.imag**2)
        max_index = np.argmax(magnitude[1:SAMPLES//2]) + 1
        estimated_freq = max_index * ACTUAL_SAMPLE_RATE / SAMPLES
        print("主频约为: {} Hz".format(estimated_freq))
        display.text(font2, "MAIN FREQ:", CENTER_X - 100, BAR_Y - 30, st7789.WHITE)
        text1 = str(estimated_freq) + "Hz"
        display.text(font2, text1, CENTER_X - 100, BAR_Y, st7789.WHITE)
五、实物演示
上述的四个状态界面分别如下：






六、后记
这次寒假按学校课程要求参加本次电子森林寒假一起练的活动，原以为比较简单，所以过年期间一直也没做，等到年后发现时间比较紧，项目内容比较粗糙，还有许多可以改进的地方，如优化采样代码提高采样率、增加更多练习难度级别、添加音阶练习模式、改进低频检测算法等。在完成本次项目时，固件烧录后死机、板载flash有限不能适应高精度音频采样，本项目采用了状态机编程的思想同时运用了中断技术，巩固了我的编程能力，加强了我的信息检索利用能力。总之非常感谢电子森林举办本次活动，通过这次活动我收获颇丰。



