import RPi.GPIO as GPIO
import time

colors = [0xFF00, 0x00FF, 0x0FF0, 0xF00F]
make_pins = (11, 12)  # PIN管脚字典

GPIO.setmode(GPIO.BOARD)        # 采用实际的物理管脚给GPIO口
GPIO.setwarnings(False)        # 去除GPIO口警告
GPIO.setup(make_pins, GPIO.OUT)        # 设置Pin模式为输出模式
GPIO.output(make_pins, GPIO.LOW)  # 设置PIN管脚为低电平 关闭LED

p_R = GPIO.PWM(make_pins[0], 2000)      # 设置频率为2KHz
p_G = GPIO.PWM(make_pins[1], 2000)
p_R.start(0)        # 初始化占空比为0(led关闭)
p_G.start(0)


def make_pwm_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def make_set_color(col):
    r_val = col >> 8
    g_val = col & 0x00FF
    r_val = make_pwm_map(r_val, 0, 255, 0, 100)
    g_val = make_pwm_map(g_val, 0, 255, 0, 100)
    p_R.ChangeDutyCycle(r_val)
    p_G.ChangeDutyCycle(g_val)


def make_loop():
    while True:
        for col in colors:
            make_set_color(col)
            time.sleep(0.5)


def make_destroy():
    p_G.stop()
    p_R.stop()
    GPIO.output(make_pins, GPIO.LOW)
    GPIO.cleanup()


if __name__ == "__main__":
    try:
        make_loop()
    except KeyboardInterrupt:
        make_destroy()
