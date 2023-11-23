import gpiozero
import time
import RPi.GPIO as GPIO
#import lgpio

steps_per_revolution = 200

GPIO.setmode(GPIO.BCM)

x_step = GPIO.setup(3, GPIO.OUT)
y_step = GPIO.setup(5, GPIO.OUT)
z_step = GPIO.setup(7, GPIO.OUT)

x_dir = GPIO.setup(11, GPIO.OUT)
y_dir = GPIO.setup(13, GPIO.OUT)
z_dir = GPIO.setup(15, GPIO.OUT)

enable = GPIO.setup(19, GPIO.OUT)


while True:
    x_dir.on()
    y_dir.on()
    z_dir.on()

    for i in range(0, 200):
        GPIO.output(x_step, GPIO.HIGH)
        time.sleep(0.500)
        GPIO.output(x_step, GPIO.LOW)
        time.sleep(0.500)
        

