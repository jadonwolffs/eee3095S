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
    #status = False         
    GPIO.setmode(GPIO.BOARD)         
    GPIO.setup(40, GPIO.OUT)
    GPIO.setup(38, GPIO.OUT)
    GPIO.setup(36, GPIO.OUT)
    
    GPIO.setup(5, GPIO.IN) #PB1        
    GPIO.add_event_detect(5, GPIO.RISING, bouncetime=300)     
    GPIO.add_event_callback(5,buttonOneHandler)
    
    GPIO.setup(3, GPIO.IN) #PB2
    GPIO.add_event_detect(3, GPIO.RISING, bouncetime=300)         
    GPIO.add_event_callback(3,buttonTwoHandler)
    
    GPIO.output(40,GPIO.LOW)
    while True:
        pass

# handle the button event
def buttonOneHandler (pin):
    global status
    global count
    count=count+1
    print ("handling button1 event ",count)

    # turn the green LED off
    if status:
        GPIO.output(40,GPIO.LOW)
        GPIO.output(38,GPIO.LOW)
        GPIO.output(36,GPIO.LOW)
        status = False
    # turn the green LED on
    else:
        GPIO.output(40,GPIO.HIGH)
        GPIO.output(38,GPIO.HIGH)
        GPIO.output(36,GPIO.HIGH)
        status = True

def buttonTwoHandler (pin):     
    global status
    global count
    count=count-1
    print ("handling button2 event ", count)      
    
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
        status = False
        count = 0
        while True:
            main()
    except KeyboardInterrupt:
        print("Exiting gracefully")
        # Turn off your GPIOs here
        GPIO.cleanup()
    except e:
        print("Some other error occurred")
        print(e.message)
        GPIO.cleanup()

# GPIO.add_event_detect(BTN_B, GPIO.RISING, method_on_interrupt)
# GPIO.add_event_detect(BTN_PIN, GPIO.FALLING, callback=callback_method(),bouncetime=300)
