import time
import gpiozero


steps_per_revolution = 1600  # This might change based on your setup and microstepping settings
delay_between_steps = 0.0025  # This controls the speed, smaller values = faster speed

x_step = gpiozero.LED(2)  # Step pin
x_dir = gpiozero.LED(17)  # Direction pin
enable = gpiozero.LED(10)  # Enable pin

# Function to make one step
def step():
    x_step.on()
    time.sleep(delay_between_steps)
    x_step.off()
    time.sleep(delay_between_steps)

# Enable the motor
enable.on()

# Set the direction (True/False depending on your setup)
x_dir.value = True  # Change to False to reverse direction

# Make the motor spin
for _ in range(steps_per_revolution):
    step()

# Disable the motor
enable.off()
