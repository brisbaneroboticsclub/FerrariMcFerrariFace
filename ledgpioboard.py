import RPi.GPIO as GPIO
import time

'''
http://brisbaneroboticsclub.id.au/ferrari-mcferrari-face/

To run code
$ python3 ledgpioboard.py
'''


GPIO.cleanup()

GPIO.setmode(GPIO.BOARD)
#GPIO.setwarnings(False)

GPIO.setup(40,GPIO.OUT)
print("LED on")
#GPIO.output(21,GPIO.HIGH)
GPIO.output(40,True)

time.sleep(1)

while 1:
    try:
        print( "LED on")
        #GPIO.output(21,GPIO.HIGH)
        GPIO.output(40,True)

        time.sleep(1)
        print("LED off")
        #GPIO.output(21,GPIO.LOW)
        GPIO.output(40,False)
    except KeyboardInterrupt:
        print('exiting')
        break

GPIO.cleanup()