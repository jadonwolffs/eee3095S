#!/usr/bin/python3
"""
Python Practical Template
Keegan Crankshaw
Readjust this Docstring as follows:
Names: Jadon Wolffs
Student Number: WLFJAD001
Prac: 1
Date: 20/07/2019
"""

# import Relevant Librares
import RPi.GPIO as GPIO
import time

# Logic that you write
def main():
    time.sleep(.1);

# handle the button event
def buttonEventHandler (pin):
    global status
    print ("handling button event")

    # turn the green LED off
    if status:
        GPIO.output(40,GPIO.LOW)
        status = False
    # turn the green LED on
    else:
        GPIO.output(40,GPIO.HIGH)
        status = True

# Only run the functions if 
if __name__ == "__main__":
    # Make sure the GPIO is stopped correctly
    try:
        #global status
        status = False
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(40, GPIO.OUT)
        GPIO.setup(5, GPIO.IN)
        GPIO.add_event_detect(5, GPIO.RISING, bouncetime=300)
        GPIO.add_event_callback(5,buttonEventHandler)
        GPIO.output(40,GPIO.LOW)
        while True:
            main()
    except KeyboardInterrupt:
        print("Exiting gracefully")
        # Turn off your GPIOs here
        GPIO.cleanup()
    #except:
        #print("Some other error occurred")
    GPIO.cleanup()

# GPIO.add_event_detect(BTN_B, GPIO.RISING, method_on_interrupt)
# GPIO.add_event_detect(BTN_PIN, GPIO.FALLING, callback=callback_method(),bouncetime=300)
