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

import RPi.GPIO as GPIO                                                         #import GPIO support library for the Pi

def main():         
    GPIO.setmode(GPIO.BOARD)         
    GPIO.setup(40, GPIO.OUT, initial=GPIO.LOW)                                  #setup pins 40, 38 and 36 to output for the LEDs
    GPIO.setup(38, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(36, GPIO.OUT, initial=GPIO.LOW)
    
    GPIO.setup(5, GPIO.IN)                                                      #set pin 5 to input for the first pushbutton        
    GPIO.add_event_detect(5, GPIO.RISING, bouncetime=300)                       #add an interrupt to pin 5 for the first button, detects the rising edge and is debounced in a threshold of 300ms
    GPIO.add_event_callback(5,buttonOneHandler)
    
    GPIO.setup(3, GPIO.IN)                                                      #set pin 3 to input for the second pushbutton
    GPIO.add_event_detect(3, GPIO.RISING, bouncetime=300)                       #add an interrupt to pin 3 for the second button, detects the rising edge and is debounced in a threshold of 300ms
    GPIO.add_event_callback(3,buttonTwoHandler)
    
    GPIO.output(40,GPIO.LOW)                                                    #set the starting position of the led pins to off
    GPIO.output(38,GPIO.LOW)
    GPIO.output(36,GPIO.LOW)

    while True:                                                                 #a busy wait loop that gets interrupted on a button press
        pass                                                                    #anything that needed to happen in the constant running of the programme would happen here

def buttonOneHandler (pin_num):                                                 #function which handles the first button being pressed (increment)
    global count                                                                #force the function to us the global count variable
    
    if count==7:                                                                #make sure it is not incremented past 7
        count = 0
    else:
	    count+=1                                                               	#increment the count variable
    
    bincount = '{0:03b}'.format(count)                                          #convert the count to a binary number and format it into 3 digits (leading 0s)
    print ("incrementing ","count to ",count," with binary value ", bincount)   #print trace statement
    update_leds()                                                               #call the update function to update the leds based on the new count

def buttonTwoHandler (pin_num):                                                 #function which handles the second button being pressed (decrement)
    global count                                                                #force the function to us the global count variable
    
    if count==0:                                                               	#make sure it is not decremented past 0
        count = 7
    else:
	    count-=1                                                               	#decrement the count variable
    
    bincount = '{0:03b}'.format(count)                                          #convert the count to a binary number and format it into 3 digits (leading 0s)
    print ("decrementing ","count to ",count," with binary value ", bincount)   #print trace statement
    update_leds()                                                               #call the update function to update the leds based on the new count

def update_leds():
    global bincount
    bincount = '{0:03b}'.format(count)                                          #convert the count to a binary number and format it into 3 digits (leading 0s)
    if bincount[0:-2]=='1':                                                     #check first digit
        GPIO.output(40,GPIO.HIGH)                                               #turn on led 1
    else:         
        GPIO.output(40,GPIO.LOW)                                                #turn off led 1
    if bincount[1:-1]=='1':                                                     #check second digit
        GPIO.output(38,GPIO.HIGH)                                               #turn on led 2
    else:         
        GPIO.output(38,GPIO.LOW)                                                #turn off led 2
    if bincount[2:3]=='1':                                                      #check third digit
        GPIO.output(36,GPIO.HIGH)                                               #turn on led 3
    else:
        GPIO.output(36,GPIO.LOW)                                                #turn off led 2
 
if __name__ == "__main__":                                                      #make sure that the program is run directly
    try:
        count = 0                                                               #set count to 0 initially
        while True:                                                             #keep running the main method on finish (mainly by convention)
            main()
    except KeyboardInterrupt:                                                   #catch the keyboard interrupt and shut the GPIO down cleanly
        print("Exiting gracefully on interrupt")
        GPIO.cleanup()
    except e:                                                                   #catch any other errors and still attempt to shut down cleanly
        print("Some other error occurred")
        print(e.message)
        GPIO.cleanup()
