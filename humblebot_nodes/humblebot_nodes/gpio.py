import time
#import RPi.GPIO as GPIO
from gpiozero import LED

steps_per_revolution = 200

x_step = LED(2)

x_dir = LED(17)

enable = LED(10)


while True:
    time.sleep(10)
    enable.on()
    print("on")
    time.sleep(10)
    enable.off()
    print("off")    

        
