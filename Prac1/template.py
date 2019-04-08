#!/usr/bin/python3
"""
Python Practical Template
Keegan Crankshaw
Readjust this Docstring as follows:
Names: Jadon Wolffs
Student Number: WLFJAD001
Prac: 1
Date: dd/07/2019
"""

# import Relevant Librares
import RPi.GPIO as GPIO

# Logic that you write
def main():
    print("write your logic here")


# Only run the functions if 
if __name__ == "__main__":
    # Make sure the GPIO is stopped correctly
    try:
        while True:
            main()
    except KeyboardInterrupt:
        print("Exiting gracefully")
        # Turn off your GPIOs here
        GPIO.cleanup()
    except:
        print("Some other error occurred")

# GPIO.add_event_detect(BTN_B, GPIO.RISING, method_on_interrupt)
# GPIO.add_event_detect(BTN_PIN, GPIO.FALLING, callback=callback_method(),bouncetime=300)
