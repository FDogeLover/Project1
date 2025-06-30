import serial
import cv2
import time
import RPi.GPIO as GPIO
from threading import Thread
from test_class_oled import *


class GpioCtrl:
    def __init__(self):
        self.oled = OledCtrl()
        self.oled.init_oled()
        self.KEY = 19
        self.DUOJI = 6
        self.LED_BLUE = 22
        self.LED_RED = 17
        self.LED_GREEN = 27
        self.LASER_1 = 4
        self.LASER_2 = 23
        # self.pwm_sg90 = GPIO.PWM(self.DUOJI, 50)

        self.flag_gpio_has_released = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        #         GPIO.setup(self.KEY, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 解决输入引脚处于悬空状态，引脚的值将是漂动的
        #         GPIO.add_event_detect(self.KEY, GPIO.BOTH, bouncetime=50)
        GPIO.setup(self.LASER_1, GPIO.OUT)
        GPIO.setup(self.LASER_2, GPIO.OUT)
        GPIO.setup(self.LED_RED, GPIO.OUT)
        GPIO.setup(self.LED_GREEN, GPIO.OUT)
        GPIO.setup(self.LED_BLUE, GPIO.OUT)
        GPIO.setup(self.DUOJI, GPIO.OUT)
        #         GPIO.setup(2, GPIO.OUT)

        pass

    def init_gpio(self):

        # GPIO.setmode(GPIO.BCM)
        # GPIO.setwarnings(False)
        #         self.oled.init_oled()
        time.sleep(0.2)
        GPIO.output(self.LASER_1, GPIO.HIGH)
        GPIO.output(self.LASER_2, GPIO.HIGH)
        GPIO.output(self.LED_RED, GPIO.LOW)
        GPIO.output(self.LED_GREEN, GPIO.LOW)
        GPIO.output(self.LED_BLUE, GPIO.LOW)
        #         self.pwm_sg90 = GPIO.PWM(self.DUOJI, 50)
        #         self.oled.init_oled()
        time.sleep(0.3)

    def init_key(self):
        GPIO.setup(self.KEY, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 解决输入引脚处于悬空状态，引脚的值将是漂动的
        GPIO.add_event_detect(self.KEY, GPIO.BOTH, bouncetime=50)

    def init_sg90(self):
        self.pwm_sg90 = GPIO.PWM(self.DUOJI, 50)
        self.pwm_sg90.start(2.5)  # 转到0度
        time.sleep(1.5)

    def key_mode(self):
        GPIO.setmode(GPIO.BCM)
        #         GPIO.setup(self.KEY, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 解决输入引脚处于悬空状态，引脚的值将是漂动的
        #         GPIO.add_event_detect(self.KEY, GPIO.BOTH, bouncetime=50)
        self.oled.show_info(2, "Waiting for key")
        num_key = 0
        while True:
            if GPIO.event_detected(self.KEY) and GPIO.input(self.KEY) == GPIO.HIGH:
                self.led_set_green(0.3)
                num_key += 1
                str_show = "num of key: " + str(num_key)
                self.oled.show_info(2, str_show)
                time_now = time.time()
                while True:
                    if time.time() - time_now >= 4:
                        break
                    if GPIO.event_detected(self.KEY) and GPIO.input(self.KEY) == GPIO.HIGH:
                        self.led_set_green(0.2)
                        num_key += 1
                        str_show = "num of key: " + str(num_key)
                        self.oled.show_info(2, str_show)
                    time.sleep(0.1)
                if num_key == 2:
                    self.oled.show_info(2, "fly mode: 1")
                elif num_key == 1:
                    self.oled.show_info(2, "fly mode: 2")
                else:
                    self.oled.show_info(2, "fly mode set error")
                return num_key
            time.sleep(0.1)

    def key_fly(self):
        self.oled.show_info(3, "Ready to fly")
        self.led_set_blue(2)
        time.sleep(2)
        while True:
            if GPIO.event_detected(self.KEY) and GPIO.input(self.KEY) == GPIO.HIGH:
                self.led_set_red(0.3)
                self.oled.show_info(3, "About to take off")
                time.sleep(2)
                break
            time.sleep(0.2)

    def laser_set_1(self, set_time):
        GPIO.output(self.LASER_1, GPIO.LOW)
        time.sleep(set_time)
        GPIO.output(self.LASER_1, GPIO.HIGH)

    def laser_set_2(self, set_time):
        GPIO.output(self.LASER_2, GPIO.LOW)
        time.sleep(set_time)
        GPIO.output(self.LASER_2, GPIO.HIGH)

    def led_set_red(self, set_time):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.LED_RED, GPIO.OUT)

        GPIO.output(self.LED_RED, GPIO.HIGH)
        time.sleep(set_time)
        GPIO.output(self.LED_RED, GPIO.LOW)

    def led_set_green(self, set_time):
        GPIO.output(self.LED_GREEN, GPIO.HIGH)
        time.sleep(set_time)
        GPIO.output(self.LED_GREEN, GPIO.LOW)

    def led_set_blue(self, set_time):
        GPIO.output(self.LED_BLUE, GPIO.HIGH)
        time.sleep(set_time)
        GPIO.output(self.LED_BLUE, GPIO.LOW)

    def duoji_set(self, angle_mode):
        #         self.pwm_sg90.start(2.5)  # 转到0度
        #         time.sleep(1.5)
        if angle_mode == 1:  # 90度
            self.pwm_sg90.ChangeDutyCycle(7.5)
            time.sleep(0.5)
            # self.pwm_sg90.ChangeDutyCycle(7.5)
        if angle_mode == 2:  # 180度
            self.pwm_sg90.ChangeDutyCycle(12)
            time.sleep(0.5)
            # self.pwm_sg90.ChangeDutyCycle(7.5)
        if angle_mode == 3:  # 转到0度
            self.pwm_sg90.ChangeDutyCycle(2.5)
            time.sleep(0.5)
        pass

    def camera_spin(self, mode, servo_gpio=6):
        G = servo_gpio
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        def set_angle(angle):
            GPIO.setup(G, GPIO.OUT)
            pwm = GPIO.PWM(G, 50)
            pwm.start(0)
            duty = 2.5 + (angle / 180) * 10
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.7)
            pwm.stop()

        if mode == 'left':
            set_angle(5)
        elif mode == 'right':
            set_angle(180)
        else:
            print("Mode Error!Please give the right command!")
    
    def release(self):
        if not self.flag_gpio_has_released:
            GPIO.cleanup()
            self.oled.show_info(3, "gpio has released")
            self.flag_gpio_has_released = True

    def __del__(self):
        if not self.flag_gpio_has_released:
            GPIO.cleanup()
            self.oled.show_info(3, "gpio has released")
            self.flag_gpio_has_released = True
