import uos
from ulab import numpy as np
#from ulab import scipy as spy
import gc
import test.st7789 as st7789
from test.fonts import vga2_8x8 as font1
from test.fonts import vga1_16x32 as font2
import random
import framebuf
from machine import Pin, SPI, ADC,PWM,I2C
import time, math,array
from utime import sleep_ms   #sleep_ms()
import utime  #utime.sleep_us()
from test.buzzer_music import music
from test.buzzer_music import tones
from test import mma7660
import struct
song='0 B4 1 43 0.6456692814826965;2 B4 1 43 0.7716535329818726;3 A4 1 43 0.8661417365074158;5 B4 1 43 0.9370078444480896;7 D5 1 43;9 B4 1 43;11 F#4 1 43;12 A4 2 43;14 B4 1 43;16 B4 1 43;18 B4 1 43;19 A4 1 43;21 B4 1 43;23 F5 1 43;25 E5 1 43;27 D5 1 43;28 A4 2 43;30 B4 1 43;32 B4 1 43;34 B4 1 43;35 A4 1 43;37 B4 1 43;39 D5 1 43;41 B4 1 43;43 F#4 1 43;44 A4 2 43;46 B4 1 43;48 B4 1 43;50 B4 1 43;52 A4 1 43;53 B4 1 43;62 B4 1 43;64 B4 1 43;66 B4 1 43;67 A4 1 43;69 B4 1 43;71 D5 1 43;73 B4 1 43;75 F#4 1 43;76 A4 2 43;78 B4 1 43;80 B4 1 43;82 B4 1 43;83 A4 1 43;85 B4 1 43;87 F5 1 43;89 E5 1 43;91 D5 1 43;92 A4 2 43;94 B4 1 43;96 B4 1 43;98 B4 1 43;99 A4 1 43;101 B4 1 43;103 D5 1 43;105 B4 1 43;107 F#4 1 43;108 A4 2 43;110 B4 1 43;112 B4 1 43;114 B4 1 43;115 A4 1 43;117 B4 1 43;119 F5 1 43;121 E5 1 43;123 D5 1 43;124 A4 2 43;126 B4 1 43;128 B4 1 43;130 B4 1 43;131 A4 1 43;133 B4 1 43;135 D5 1 43;137 B4 1 43;139 F#4 1 43;140 A4 2 43;142 B4 1 43;144 B4 1 43;146 B4 1 43;147 A4 1 43;149 B4 1 43;151 F5 1 43;153 E5 1 43;155 D5 1 43;156 A4 2 43;158 B4 1 43;160 B4 1 43;162 B4 1 43;163 A4 1 43;165 B4 1 43;167 D5 1 43;169 B4 1 43;171 F#4 1 43;172 A4 2 43;174 B4 1 43;176 B4 1 43;178 B4 1 43;179 A4 1 43;180 B4 1 43;181 D5 1 43;183 F5 1 43;185 E5 1 43;187 D5 1 43;188 A4 2 43'

machine.freq(125000000) # 锁定在 125MHz

#image_file0 = "/logo.bin" #图片文件地址

sda=Pin(10)
scl=Pin(11)
mma7660=machine.I2C(1,sda=sda,scl=scl,freq=400000)
mma7660.writeto_mem(76,7,b'1')

'''
f_image = open(image_file0, 'rb')
 
for column in range(1,240):
                buf=f_image.read(480)
                display.blit_buffer(buf, 1, column, 240, 1)
'''

t=0
# 频率表字典：音名 -> 频率 (Hz)
PRACTICE_MODE_NOTE_FREQ = {
    # 第 3 组 (低音区)
    "C3": 130.81,  "D3": 146.83,  "E3": 164.81, "F3": 174.61,
     "G3": 196.00,  "A3": 220.00,  "B3": 246.94,
    
    # 第 4 组 (中音区 / 中央C 所在组)
    "C4": 261.63,  "D4": 293.66,  "E4": 329.63, "F4": 349.23,
     "G4": 392.00,  "A4": 440.00,  "B4": 493.88,
    
    # 第 5 组 (高音区)
    "C5": 523.25,  "D5": 587.33,  "E5": 659.25, "F5": 698.46,
     "G5": 783.99,  "A5": 880.00, "B5": 987.77
}


###练习模式的显示模块
###方框条
#1. 初始化屏幕显示
st7789_res = 0
st7789_dc  = 1
disp_width = 240
disp_height = 240
CENTER_Y = int(disp_width/2)
CENTER_X = int(disp_height/2)
spi_sck=Pin(2)
spi_tx=Pin(3)
spi0=SPI(0,baudrate=4000000, phase=1, polarity=1, sck=spi_sck, mosi=spi_tx)

display = st7789.ST7789(spi0,
                        disp_width,
                        disp_width,
                        reset=machine.Pin(st7789_res, machine.Pin.OUT),
                        dc=machine.Pin(st7789_dc, machine.Pin.OUT),
                        xstart=0,
                        ystart=0,
                        rotation=0)
#display.fill(st7789.BLACK)
#display.text(font2, "EETREE", 10, 10)
#display.text(font2, "www.eetree.cn", 10, 40)

# 2. 颜色与常量定义
BG_COLOR = st7789.BLACK
BASE_LINE_COLOR = st7789.WHITE
COLOR_PERFECT = st7789.GREEN
COLOR_FLAT = st7789.BLUE     # 偏低显示蓝色
COLOR_SHARP = st7789.RED     # 偏高显示红色

# UI 坐标配置

BAR_Y = 100                  # 方框条的 Y 坐标
BAR_H = 40                   # 方框条的高度

benchmark_freq = PRACTICE_MODE_NOTE_FREQ["C5"]             # 基准音 C4
benchmark_note = "C5"
MAX_CENTS = 50               # 屏幕单侧最大显示 50 音分 (即正负半个音

# 记录上一次绘制的方框信息，用于局部擦除
last_bar_x = CENTER_X
last_bar_w = 0

def init_practice():
    """初始化静态界面"""
    display.fill(BG_COLOR)
    # 绘制居中的基准线
    display.vline(CENTER_X, BAR_Y - 10, BAR_H + 20, BASE_LINE_COLOR)
    display.text(font2, benchmark_note, CENTER_X - 10, BAR_Y + 50, st7789.WHITE)

def update_tuner_bar(freq):
    """根据输入的频率更新动态方框条"""
    global last_bar_x, last_bar_w
    
    if freq <= 0:
        return

    # A. 计算音分差 (MicroPython 没有 log2，使用换底公式)
    cents = 1200 * math.log(freq / benchmark_freq) / math.log(2)
    
    # B. 限制在最大显示范围内，并映射到屏幕像素
    cents = max(-MAX_CENTS, min(MAX_CENTS, cents))
    px_per_cent = (display.width // 2) / MAX_CENTS
    bar_width = int(cents * px_per_cent)

    # C. 确定方框颜色 (误差在 5 音分以内显示绿色)
    if abs(cents) <= 5:
        color = COLOR_PERFECT
    elif cents > 0:#高：红色
        color = COLOR_SHARP
    else:#低：蓝色
        color = COLOR_FLAT

    # D. 局部擦除旧方框 (避免全屏清屏导致的闪烁)
    if last_bar_w > 0:
        display.fill_rect(last_bar_x, BAR_Y, last_bar_w, BAR_H, BG_COLOR)

    # E. 计算新方框坐标并绘制（fill_rect函数特性，只能从左往右，从上往下，不能反方向，即wide和height不能为负数）
    if bar_width > 0:
        new_x = CENTER_X
        new_w = bar_width
    elif bar_width < 0:    #矩形条平移
        new_x = CENTER_X + bar_width
        new_w = -bar_width
    else:
        new_x = CENTER_X
        new_w = 0

    if new_w > 0:
        display.fill_rect(new_x, BAR_Y, new_w, BAR_H, color)

    # F. 更新状态并重绘被擦除的基准线
    last_bar_x = new_x
    last_bar_w = new_w
    display.vline(CENTER_X, BAR_Y - 10, BAR_H + 20, BASE_LINE_COLOR)
 
def freq_to_pitch(measured_f):
    """
    将输入频率转换为音名和偏差音分
    返回: (音名字符串, 标准频率, 音分偏差)
    """
    if measured_f <= 0:
        return "None", 0, 0
    
    # 1. 寻找最接近的标准音
    closest_note = "C4"
    min_diff = float('inf')
    
    for note, std_f in PRACTICE_MODE_NOTE_FREQ.items():
        diff = abs(measured_f - std_f)
        if diff < min_diff:
            min_diff = diff
            closest_note = note
            
    target_std_f = PRACTICE_MODE_NOTE_FREQ[closest_note]
    
    # 2. 计算音分差 (Cents)
    # 公式: 1200 * log2(f_measured / f_std)
    # math.log(x, 2) 在某些 MicroPython 版本中不可用，使用换底公式
    cents = 1200 * (math.log(measured_f / target_std_f) / math.log(2))
    
    return closest_note, target_std_f, cents  
 
###采样音频信号部分
###使用fft算法

#1. 初始化设置
sound_adc=ADC(Pin(26))

# 2. 采样参数设置
SAMPLES = 512      # 采样点数 
SAMPLE_RATE = 8000    # 目标采样率 8kHz
ACTUAL_SAMPLE_RATE=4200   #测量实际采样率4kHZ
interval_us = int(1000000 / SAMPLE_RATE)

def collect_samples(n):
    # 创建一个空数组用于存储原始 ADC 值
    # RP2040 的 ADC 是 12 位，读取值范围 0-65535

    raw_data = np.zeros(n, dtype=np.uint16)
    start = utime.ticks_us()
    for i in range(n):
        raw_data[i] = sound_adc.read_u16()
        utime.sleep_us(interval_us)   #修正误差
        
    end = utime.ticks_us()
    print("实际平均采样率:", 1000000 / ((end - start) / n))

    return raw_data
    
#3. 获取并转换数据
n = np.linspace(0, SAMPLES - 1, num=SAMPLES)
window = 0.5 - 0.5 * np.cos(2 * 3.14159265 * n / (SAMPLES - 1))
    
###模式切换部分
###通过按键和摇杆

#分配几个状态
menu=0
practice=1
success=2
measure=3
state=menu   #初始化状态
#参数列表
score=0 

# 1. 硬件引脚配置
# 使用 GPIO 6，设置为输入模式并启用内部上拉电阻
buttonA = Pin(6 , Pin.IN, Pin.PULL_UP)
buttonB = Pin(5, Pin.IN, Pin.PULL_UP) 
buttonStart = Pin(7,Pin.IN, Pin.PULL_UP) 
buttonSelect = Pin(8,Pin.IN, Pin.PULL_UP)

#标志列表
StartA = False
StartB = False
Select = False
init_menu_sign = False
# 2. 中断回调函数
last_interrupt_time0 = 0

def button_handler(pin):
    global last_interrupt_time0 , StartA , state ,score, Select,init_menu_sign,StartB
    # 获取当前毫秒数
    current_time0 = utime.ticks_ms()
    # 计算两次中断之间的时间差
    # 如果两次按下的间隔小于 200ms，则判定为抖动，直接返回
    if utime.ticks_diff(current_time0, last_interrupt_time0) < 200:
        return
    # 更新最后一次有效触发的时间
    last_interrupt_time0 = current_time0
    if pin == buttonA:
        print(pin)
        if state == success:
         StartA = True
        
         
    elif pin == buttonB:
        print(pin)
        if state == practice:
         score+=1
        if state == success:
         StartB = True
        
    elif pin == buttonSelect:
        print(pin)
        if state == menu:
         Select=True
    elif pin == buttonStart:
        print(pin)
        state = menu
        init_menu_sign=True
        

# 3. 中断触发配置
# trigger=Pin.IRQ_FALLING 表示在按键按下（电平从高变低）时触发
buttonA.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonB.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonStart.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
buttonSelect.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
def init_bg():
    """初始化静态界面"""
    display.fill(BG_COLOR)
def init_menu():
    init_bg()
def init_measure():
    init_bg()
    
init_menu()
MODE_select_button=0

#初始化摇杆的ADC
xAxis = ADC(Pin(28))
yAxis = ADC(Pin(29))
# 定义摇杆触发阈值（ADC 范围 0-65535，中间值约 32768）
UP_THRESHOLD = 50000   # 向上推的阈值
DOWN_THRESHOLD = 15000 # 向下推的阈值

# 状态变量：用于防止摇杆推住不动时菜单疯狂滚动
joy_moved = False

#姿态检测
# 姿态追踪变量
i2c = I2C(1, scl=Pin(11), sda=Pin(10), freq=400000)

# 初始化全局变量
initial_x, initial_y, initial_z = 0, 0, 0
is_tilted = False  # 标志位：当前是否处于倾斜状态
tilt_threshold = 25 # 阈值：MMA7660 范围约 -32~31。25 左右代表接近 90 度翻转

def calibrate_tilt():
    """在进入练习模式前调用一次，记录初始姿态"""
    global initial_x, initial_y, initial_z
    data = i2c.readfrom_mem(0x4C, 0x00, 3)
    
    def conv(val):
        val &= 0x3F
        if val & 0x20: val -= 64
        return val

    initial_x = conv(data[0])
    initial_y = conv(data[1])
    initial_z = conv(data[2])
    print(f"校准完成！初始姿态: X={initial_x}, Y={initial_y}，Z={initial_z}")

def check_tilt_change():
    global initial_x, initial_y, is_tilted, benchmark_note, benchmark_freq
    
    # 1. 读取当前 X, Y, Z
    data = i2c.readfrom_mem(0x4C, 0x00, 3)
    def conv(val):
        val &= 0x3F
        if val & 0x20: val -= 64
        return val
    
    curr_x = conv(data[0])
    curr_y = conv(data[1])
    
    # 2. 计算相对于初始位置的偏差
    diff_x = abs(curr_x - initial_x)
    diff_y = abs(curr_y - initial_y)
    
    # 3. 核心逻辑判断
    # 如果 X 或 Y 的变化量超过阈值，说明翻转了接近 90 度
    if (diff_x > tilt_threshold or diff_y > tilt_threshold):
        if not is_tilted:  # 如果之前是水平的，现在刚倾斜
            is_tilted = True
            
            # --- 执行切题逻辑 ---
            notes_list = list(PRACTICE_MODE_NOTE_FREQ.keys())
            benchmark_note = random.choice(notes_list)
            benchmark_freq = PRACTICE_MODE_NOTE_FREQ[benchmark_note]
            
            play_target_hint(benchmark_note)
            print(f"检测到翻转 90 度！新目标: {benchmark_note}")
            return True
    else:
        # 如果偏差很小，说明回到了初始位置（或接近水平）
        if is_tilted:
            print("设备已回正")
        is_tilted = False 
        
    return False
calibrate_tilt()  #先记录上电时的初始方向
# 摇晃追踪变量
# 记录上一次摇晃的时间，防止连续触发
last_shake_time = 0
def check_shake():
    global last_shake_time, state, score, StartA
    
    # 限制触发频率：1秒内只准摇一次
    current_time = utime.ticks_ms()
    if utime.ticks_diff(current_time, last_shake_time) < 1000:
        return False

    # 读取 X, Y, Z 三轴寄存器 (地址 0, 1, 2)
    # MMA7660 地址 0x4C
    data = i2c.readfrom_mem(0x4C, 0x00, 3)
    
    # 转换 6 位补码的函数
    def conv(val):
        val &= 0x3F
        if val & 0x20: val -= 64
        return val

    x = conv(data[0])
    y = conv(data[1])
    z = conv(data[2])

    # 计算晃动强度 (三轴绝对值相加)
    # 正常静止时总和大约在 20 左右（重力加速度）
    # 剧烈摇晃时这个数值会激增
    shake_force = abs(x) + abs(y) + abs(z)

    # 阈值设定：根据实际手感调整，通常 45-60 比较灵敏
    if shake_force > 45:
        last_shake_time = current_time
        print("检测到摇一摇！重新开始...")
        # --- 重置逻辑 ---
        score = 0
        state = practice  # 回到练习模式
        # 触发重新选题
        # 1. 获取所有键（音名）并转换为列表
        notes_list = list(PRACTICE_MODE_NOTE_FREQ.keys())
        # 2. 随机选取一个键
        benchmark_note = random.choice(notes_list)
        # 3. 根据选取的键获取对应的频率
        benchmark_freq = PRACTICE_MODE_NOTE_FREQ[benchmark_note]
        #播放提示音
        #play_target_hint(benchmark_note)
        #print(f"重新开始，切换至目标音: {benchmark_note}")
        return True
        
    return False

#蜂鸣器提示音
pwm = PWM(Pin(19))
mySong = music(song, pins=[Pin(23)])
pwm.freq(50)
# 初始化蜂鸣器 PWM
buzzer = PWM(Pin(23))
buzzer.duty_u16(0) # 初始关闭
def play_target_hint(note_name):
    try:
        if note_name in tones:
            freq = tones[note_name]
            buzzer.freq(freq)
            buzzer.duty_u16(32768) # 50% 占空比
            utime.sleep_ms(300)    # 短促响一声
            buzzer.duty_u16(0)     # 关闭
    except Exception as e:
        print("播放提示音失败:", e)
        
##引入时间戳
# 记录符合标准开始的时间点（0 表示当前不符合或刚重置）
match_start_time = 0 
# 标志位：防止在两秒达成后瞬间多次加分
is_scored_for_this_note = False
#记录最高分数
highest_score = 0
#标志位：继续练习
continue_sign = False
while True:
    if init_menu_sign == True: #清屏标志
        init_menu()
        init_menu_sign=False
        continue_sign=False
        
    if state==menu:     #状态判断1
     '''
     x=mma7660.readfrom_mem(76,0,8)
     y=mma7660.readfrom_mem(76,1,8)
     z=mma7660.readfrom_mem(76,2,8)
     xout=struct.unpack('<h',x)[0]
     yout=struct.unpack('<h',y)[0]
     zout=struct.unpack('<h',z)[0]
     '''
     
     display.text(font2, "MAIN MENU", 60, 40, st7789.YELLOW)
     ##轮询摇杆方向
     # 1. 读取摇杆当前数值
     x_val = xAxis.read_u16()
     #print(x_val)
     # 2. 判断摇杆动作
     if x_val > UP_THRESHOLD and not joy_moved:
         # 向上推：切换到下一个选项
         MODE_select_button = 1
         joy_moved = True # 锁定，直到回中
     elif x_val < DOWN_THRESHOLD and not joy_moved:
         # 向下推：切换到上一个选项
         MODE_select_button = 0
         joy_moved = True # 锁定
     elif x_val > 25000 and x_val < 40000:
         # 摇杆回到中间区域（给中间留出缓冲区）
         joy_moved = False
         
     # 选项 1: 练习模式
     color1 = st7789.BLUE if MODE_select_button == 0 else st7789.WHITE
     display.rect(40, 80, 160, 40, color1) # 画边框
     display.text(font1, "PRACTICE MODE", 55, 92, color1)

     # 选项 2: 测量模式
     color2 = st7789.BLUE if MODE_select_button == 1 else st7789.WHITE
     display.rect(40, 140, 160, 40, color2) # 画边框
     display.text(font1, "MEASURE MODE", 60, 152, color2)
    
     # 在选中的项旁边画一个小箭头或方块
     pointer_y = 152 if MODE_select_button == 0 else 92
     display.fill_rect(25, pointer_y, 10, 10, st7789.BLACK)
     pointer_y = 92 if MODE_select_button == 0 else 152
     display.fill_rect(25, pointer_y, 10, 10, st7789.CYAN)
     
     if MODE_select_button==1:
         if Select==True:
             state=measure
             init_measure()
             Select=False
     elif MODE_select_button==0:
         if Select==True:
             state=practice
             score=0
             init_practice()
             Select=False
             
             
    elif state==practice:     #状态判断2
     check_tilt_change()
     print(check_shake())
     raw_samples = collect_samples(SAMPLES)
     #signal = np.array(raw_samples, dtype=np.float) - 32768      #波形数据
    
     signal = np.array(raw_samples, dtype=np.float)
     signal -= np.mean(signal)
     #window = np.extras.hanning(SAMPLES)    #不能用这个模块
     signal *= window

     #4. fft处理数据
     fft_result = np.fft.fft(signal)
     magnitude = np.sqrt(fft_result.real**2 + fft_result.imag**2)
     max_index = np.argmax(magnitude[1:SAMPLES//2]) + 1
     estimated_freq = max_index * ACTUAL_SAMPLE_RATE / SAMPLES
     print("主频约为: {} Hz".format(estimated_freq))
     closest_note, target_std_f, cents=freq_to_pitch(estimated_freq)
     display.text(font2, closest_note , CENTER_X - 10, BAR_Y -50, st7789.WHITE)
     update_tuner_bar(estimated_freq)
     text0 = "score = {}".format(score)
     text2 = "highest_score = {}".format(highest_score)
     display.text(font2, "PRECTICE MODE" , CENTER_X - 100, BAR_Y - 100, st7789.WHITE)
     display.text(font1, text2 , CENTER_X - 100, BAR_Y - 60, st7789.WHITE)
     display.text(font1, text0 , CENTER_X - 100, BAR_Y - 40, st7789.WHITE)
     display.text(font2, benchmark_note, CENTER_X - 10, BAR_Y + 50, st7789.WHITE)
     if closest_note == benchmark_note:
         # 如果是刚刚进入“符合标准”状态，记录当前时间
         if match_start_time == 0:
             match_start_time = utime.ticks_ms()
             print("音准正确，保持住...")
        
         # 计算已经坚持了多久
         duration = utime.ticks_diff(utime.ticks_ms(), match_start_time)
        
         # 如果坚持超过 2000ms 且还没加过分
         if duration >= 2000 and not is_scored_for_this_note:
             score += 1
             is_scored_for_this_note = True # 标记已得分，防止连加
         
             # 1. 获取所有键（音名）并转换为列表
             notes_list = list(PRACTICE_MODE_NOTE_FREQ.keys())
             # 2. 随机选取一个键
             benchmark_note = random.choice(notes_list)
             # 3. 根据选取的键获取对应的频率
             benchmark_freq = PRACTICE_MODE_NOTE_FREQ[benchmark_note]
             
             play_target_hint(benchmark_note)   #播放提示音
             # 得分后重置计时器和标记位，准备下一题
             match_start_time = 0
             is_scored_for_this_note = False
     else:
     # --- 新增的关键逻辑 ---
     # 只要一旦音准不对，立刻重置计时器，要求重新开始连续计时
       if match_start_time != 0:
          print("音准断了，重新开始计时")
          match_start_time = 0
          
     highest_score=max(score,highest_score)    #更新最高得分
     
     if score > 5 and continue_sign == False:
         state=success
         
    elif state == success:      #状态判断3
        display.fill(BG_COLOR)
        display.text(font2, "YOU WIN" , CENTER_X-55, BAR_Y, st7789.WHITE)
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
    elif state == measure:    #状态判断4
        display.text(font2, "MEASURE MODE" , CENTER_X - 100, BAR_Y - 100, st7789.WHITE)
        
        raw_samples = collect_samples(SAMPLES)
        #signal = np.array(raw_samples, dtype=np.float) - 32768      #波形数据
        signal = np.array(raw_samples, dtype=np.float)
        signal -= np.mean(signal)
        #window = np.extras.hanning(SAMPLES)    #不能用这个模块
        signal *= window
        #4. fft处理数据
        fft_result = np.fft.fft(signal)
        magnitude = np.sqrt(fft_result.real**2 + fft_result.imag**2)
        max_index = np.argmax(magnitude[1:SAMPLES//2]) + 1
        estimated_freq = max_index * ACTUAL_SAMPLE_RATE / SAMPLES
        print("主频约为: {} Hz".format(estimated_freq))
        display.text(font2, "MAIN FREQ:" , CENTER_X-100 , BAR_Y-30, st7789.WHITE)
        text1=str(estimated_freq)+"Hz"
        display.text(font2, text1 , CENTER_X-100 , BAR_Y , st7789.WHITE)#需要关注遮盖问题
    '''
    sound_value=sound_adc.read_u16()
    xValue = xAxis.read_u16()
    yValue = yAxis.read_u16()
    buttonValueA = buttonA.value()
    buttonValueB = buttonB.value()
    buttonValueStart = buttonStart.value()
    buttonValueSelect = buttonSelect.value()
     


    display.text(font1, "xVaule="+"%05d" %(xValue) , 10, 70)
    display.text(font1, "yVaule="+"%05d" %(yValue), 120, 70)
    display.text(font1, "buttonValueA="+str(buttonValueA), 10, 90)
    display.text(font1, "buttonValueB="+str(buttonValueB), 10, 110)
    #display.text(font1,"buttonValueStart="+str(buttonValueStart) , 10, 130)
    #display.text(font1, "buttonValueSelect="+str(buttonValueSelect), 10, 150)
    display.text(font1,"sound_value="+str(sound_value) , 10, 130)
    #display.text(font1, "buttonValueSelect="+str(buttonValueSelect), 10, 150)
    display.text(font1,"xout="+"%05d" %(xout) , 10, 170)
    display.text(font1,"yout="+"%05d" %(yout) , 10, 190)
    display.text(font1,"zout="+"%05d" %(zout) , 10, 210)
    
    #mySong.tick()
    '''
