import RPi.GPIO as GPIO
import time

'''
http://brisbaneroboticsclub.id.au/ferrari-mcferrari-face/

To run code
$ python3 pwm.py
'''

GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)

p = GPIO.PWM(11, 50)

p.start(7.5)

try:
    while True:
        print('7.5 - middle')
        p.ChangeDutyCycle(7.5)  # turn towards 90 degree
        time.sleep(1) # sleep 1 second

        print('2.5 - full right')
        p.ChangeDutyCycle(2.5)  # turn towards 0 degree
        time.sleep(1) # sleep 1 second

        print('7.5 - middle')
        p.ChangeDutyCycle(7.5)  # turn towards 0 degree
        time.sleep(1) # sleep 1 second

        print('12.5 - full left')
        p.ChangeDutyCycle(12.5) # turn towards 180 degree
        time.sleep(1) # sleep 1 second 

except KeyboardInterrupt:
    print('7.5 - middle')
    p.ChangeDutyCycle(7.5)  # turn towards 90 degree
    time.sleep(1) # sleep 1 second

    print('Stopping.')
    p.stop()
    GPIO.cleanup()