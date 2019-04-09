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

def main():         
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
    if count==8:   
        count = 0
    bincount = '{0:03b}'.format(count)
    print ("handling button1 event ","count ",count,"bincount ", bincount)
    update_leds()

def buttonTwoHandler (pin):     
    global status
    global count
    count=count-1
    if count==-1:
        count = 7
    bincount = '{0:03b}'.format(count)
    print ("handling button2 event ","count ",count,"bincount ", bincount)      
    update_leds()

def update_leds():
    global bincount
    bincount = '{0:03b}'.format(count)
    if bincount[0:-2]=='1':         
        GPIO.output(40,GPIO.HIGH)     
    else:         
        GPIO.output(40,GPIO.LOW)
    if bincount[1:-1]=='1':         
        GPIO.output(38,GPIO.HIGH)     
    else:         
        GPIO.output(38,GPIO.LOW)
    if bincount[2:3]=='1':      
        GPIO.output(36,GPIO.HIGH)
    else:
        GPIO.output(36,GPIO.LOW)

# Only run the functions if 
if __name__ == "__main__":
    try:
        status = False
        count = 0
        while True:
            main()
    except KeyboardInterrupt:
        print("Exiting gracefully on interrupt")
        GPIO.cleanup()
    except e:
        print("Some other error occurred")
        print(e.message)
        GPIO.cleanup()
