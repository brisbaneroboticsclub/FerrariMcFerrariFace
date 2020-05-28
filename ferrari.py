import RPi.GPIO as GPIO     
from time import sleep
import cv2
import threading
import evdev
from math import sin, pi

'''
http://brisbaneroboticsclub.id.au/ferrari-mcferrari-face/

To run code
$ python3 ferrari.py
'''

print('-------------------')
print('Clean up any existing GPIO settings.')
print('-------------------')

GPIO.setwarnings(False)
GPIO.cleanup()

print('-------------------')
print('Configure new GPIO settings.')
print('-------------------')

# GPIO Setup
GPIO.setmode(GPIO.BOARD)

print('-------------------')
print('Configure new Headlights.')
print('-------------------')

# Headlight Setup
GPIO.setup(40,GPIO.OUT)

print('-------------------')
print('Turn headlights on.')
print('-------------------')

GPIO.output(40,True)

print('-------------------')
print('Configuring Steering.')
print('-------------------')

# Steering setup
GPIO.setup(11, GPIO.OUT)
p = GPIO.PWM(11, 50) # Pin 8, 50 Hertz.
p.start(7.5) # 0>x>100. Start power %

#print(dir(p))

# Motor setup
in1 = 18
in2 = 16
en = 22
#temp1=1

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)

GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)

q=GPIO.PWM(en,50)
q.start(25)

# Camera setup
print('-------------------')
print('Configuring Camera.')
print('-------------------')

cap = cv2.VideoCapture(0)

def TestSteering():
    #Steering Algorithm
    while True:
        try:
            print('Press CTRL+C once to exit.')
            print('7.5 - middle')
            p.ChangeDutyCycle(7.5)  # turn towards 90 degree
            sleep(1) # sleep 1 second

            print('2.5 - full right')
            p.ChangeDutyCycle(2.5)  # turn towards 0 degree
            sleep(1) # sleep 1 second

            print('7.5 - middle')
            p.ChangeDutyCycle(7.5)  # turn towards 0 degree
            sleep(1) # sleep 1 second

            print('12.5 - full left')
            p.ChangeDutyCycle(12.5) # turn towards 180 degree
            sleep(1) # sleep 1 second
        
        except KeyboardInterrupt:
            break

def TestSteering2():

    MaxPower = 12.5 #Full left
    MinPower = 2.5 #Full right

    AvgPower = (MaxPower + MinPower)/2
    #print('Average power :',AvgPower)

    RangePower = MaxPower - MinPower
    #print('Range power :',RangePower)

    x = 0
    xstep = 12.5

    print('To EXIT, press CTRL+C once.')

    while True:
        #print('X is : ', x)
        #print('Sin x : ',(sin(x * pi/360)))
        pwr = (AvgPower + (0.5 * RangePower * (sin(x * pi/360))))
        print('Steering power is : ',pwr)

        try:
            p.ChangeDutyCycle(pwr) # turn towards 180 degree
            sleep(0.02)
        
        except KeyboardInterrupt:
            break

        x += xstep

def TestRearWheels():
    print("\n")
    print("The default speed & direction of motor is LOW & Forward.....")
    print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
    print("\n") 

    #temp1=1

    while(1):

        x=input()
        
        if x=='r':
            print("run")
            if(temp1==1):
                GPIO.output(in1,GPIO.HIGH)
                GPIO.output(in2,GPIO.LOW)
                print("forward")
                x='z'
            else:
                GPIO.output(in1,GPIO.LOW)
                GPIO.output(in2,GPIO.HIGH)
                print("backward")
                x='z'

        elif x=='s':
            print("stop")
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.LOW)
            x='z'

        elif x=='f':
            print("forward")
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)
            #temp1=1
            x='z'

        elif x=='b':
            print("backward")
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            #temp1=0
            x='z'

        elif x=='l':
            print("low")
            q.ChangeDutyCycle(25)
            x='z'

        elif x=='m':
            print("medium")
            q.ChangeDutyCycle(50)
            x='z'

        elif x=='h':
            print("high")
            q.ChangeDutyCycle(75)
            x='z'
        
        elif x=='e':
            #GPIO.cleanup()
            print("GPIO Clean up")
            break
        
        else:
            print("<<<  wrong data  >>>")
            print("please enter the defined data to continue.....")

def GetCamFrame():
    #print('Camera details :',cap)
    ret, frame = cap.read()

    return ret, frame

def doCamera():

    global ret
    global RunLoop

    while RunLoop == 'Yes':

        ret, frame = GetCamFrame()
        #print(ret)
        #print(frame)

        RowCt, ColCt, Dim = frame.shape
        #print('Col count :' + str(ColCt))
        #print('Row count :' + str(RowCt))
        frame = cv2.resize(frame, (int(0.3 * ColCt), int(0.3 * RowCt)), interpolation=cv2.INTER_CUBIC)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            #cap.release()
            cv2.destroyAllWindows()
            RunLoop = 'No'
            print('Exit requested via Camera Loop. Exiting now.')
            #break

    #cv2.destroyAllWindows()

def getCamera():

    print('-------------------')
    print('Running Camera Loop')
    print('-------------------')
    
    # Camera algorithm
    # Capture frame-by-frame

    global RunLoop

    print('To exit, click on frame and hit q.')

    RunLoop = 'Yes'

    while True and RunLoop == 'Yes':
        
        ret, frame = GetCamFrame()
        #print(ret)
        #print(frame)

        RowCt, ColCt, Dim = frame.shape
        #print('Col count :' + str(ColCt))
        #print('Row count :' + str(RowCt))
        frame = cv2.resize(frame, (int(0.3 * ColCt), int(0.3 * RowCt)), interpolation=cv2.INTER_CUBIC)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            #cap.release()
            cv2.destroyAllWindows()
            RunLoop = 'No'
            print('Exit requested via Camera Loop. Exiting now.')
            #break

    cv2.destroyAllWindows()

def ConnectXboxBT():
    print('----------------')
    print('Attempting to connect to your xbox contoller...')

    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

    print('-------------------')
    print('These devices were found on your Robot...')

    # If devices found do this or QUIT

    print ('Devices :',devices)

    if devices != []:

        for device in devices:
            print(device.path, device.name, device.phys)

        print('-------------------')
        print('Now connecting your Xbox controller....')
        print('-------------------')

        for device in devices:
            print('Path: ',device.path,', Device Name: ',device.name,', Device address: ',device.phys)
            print('-------------------')
            #print(type(device.name))
            if (device.name.__contains__('box') == True) and (device.name.__contains__('Cons') == False):
                print('-------------------')
                print('This device is compatible.')
                print('The Xbox controller was FOUND!!!')
                print('-------------------')
                print(device)
                print('-------------------')
                print(dir(device))
                print('----------')
                print(device.capabilities())
                print('----------')
                print(device.capabilities(verbose=True))
                print('----------')
                print(device.info)
                print('----------')
                print(device.leds)
                print('----------')
                print(device.name)
                print('----------')
                print(device.path)
                print('----------')
                print(device.phys)
                print('----------')
                return device
            else:
                print('-------------------')
                print('This device was not compatible.')
                print('-------------------')
    else:
        print('-------------------')
        print('Is your Xbox turned ON???')
        print('-------------------')

    print('-------------------')

def ReadXboxBT(device):
    
    print('dev is ',device)
    
    if device == 'False':
        device = ConnectXboxBT()

    print('Press any button on Xbox. Y to EXIT.')

    while 1:
        try:
            xkey = device.read_one()
            #print('key :', xkey)
            if xkey is not None:
                #print('new command-----------------------------------------------')
                print('key :', xkey)
                #print('key code :', xkey.code)
                #print('key type :', xkey.type)
                #print('key value :', xkey.value)
                if xkey.code == 304 and xkey.value == 1:
                    print('A button was pressed')
                elif xkey.code == 304 and xkey.value == 0:
                    print('A button was released')
                elif xkey.code == 305 and xkey.value == 1:
                    print('B button was pressed')
                elif xkey.code == 305 and xkey.value == 0:
                    print('B button was released')
                elif xkey.code == 306 and xkey.value == 1:
                    print('??? button was pressed')
                elif xkey.code == 306 and xkey.value == 0:
                    print('??? button was released')
                elif xkey.code == 307 and xkey.value == 1:
                    print('X button was pressed')
                elif xkey.code == 307 and xkey.value == 0:
                    print('X button was released')
                elif xkey.code == 308 and xkey.value == 1:
                    print('Y button was pressed')
                    break
                elif xkey.code == 308 and xkey.value == 0:
                    print('Y button was released')
                    break
                elif xkey.code == 311 and xkey.value == 1:
                    print('Right Trigger upper was pressed')
                elif xkey.code == 311 and xkey.value == 0:
                    print('Right Trigger upper was released')
                elif xkey.code == 313 and xkey.value == 1:
                    print('Right Trigger lower was pressed')
                elif xkey.code == 313 and xkey.value == 0:
                    print('Right Trigger lower was released')
                elif xkey.code == 310 and xkey.value == 1:
                    print('Left Trigger upper was pressed')
                elif xkey.code == 310 and xkey.value == 0:
                    print('Left Trigger upper was released')
                elif xkey.code == 10 and xkey.value == 0:
                    print('Left Trigger lower was pressed')
                elif xkey.code == 10 and xkey.value == 0:
                    print('Left Trigger lower was released')
                elif xkey.code == 0 and xkey.type == 3:  # Left joystick, left and right.
                    print('----------')
                    print('Left joystick - X axis')
                    print('value', xkey.value)
                    print('----------')
                elif xkey.code == 1 and xkey.type == 3:  # Left joystick, up and down. #Throttle ccontrol
                    print('----------')
                    print('Left joystick - Y axis')
                    print('value', xkey.value)
                    print('----------')
                elif xkey.code == 2 and xkey.type == 3:  # Right joystick, left and right. # Steering Control
                    print('----------')
                    print('Right joystick - X axis')
                    print('value', xkey.value)
                    print('----------')
                elif xkey.code == 5 and xkey.type == 3:  # Right joystick, left and right.
                    print('----------')
                    print('Right joystick - Y axis')
                    print('value', xkey.value)
                    print('----------')
                elif xkey.code == 9 and xkey.type == 3:  #
                    print('----------')
                    print('Right Trigger Lower.')
                    print('value', xkey.value)
                    print('----------')
                elif xkey.code == 10 and xkey.type == 3:  #
                    print('----------')
                    print('Left Trigger Lower.')
                    print('value', xkey.value)
                    print('----------')
                elif xkey.code == 17 and xkey.value == 0:
                    print('D-Pad was pressed')
        except:
            print('----------------')
            print('The xbox controller is not connected. Is it turned on?')
            print('----------------')
            break

def DriveManualXbox():

    print('-------------------')
    print('Running Xbox loop Loop')
    print('-------------------')

    global RunLoop

    RunLoop = 'Yes'

    #print('Y to stop, Left joystick fwd/rev, Right joystick left/right. Right Upper Trigger to Bump. Left Upper Trigger to take 25 images to video. Right trigger lower to increase bump pwm by 25.')

    device = 'False'
    
    print('-------------------')
    print('dev is ',device)
    print('-------------------')
    
    if device == 'False':
        device = ConnectXboxBT()

    Strpwr = 12.5
    p.ChangeDutyCycle(Strpwr)

    print('-------------------')
    print('GO!!!!!!!!!!!!')
    print('-------------------')

    while RunLoop == 'Yes':

        if device is not None:
            xkey = device.read_one()
            #print('key :', xkey)
            if xkey is not None:
                #print('key :',xkey)
                #print('key code :',xkey.code)
                #print('key value :',xkey.value)
                #print('key type :',xkey.type)
                if xkey.code == 304 and xkey.value == 1:
                    #print('A button was pressed')
                    pass
                #elif xkey.code == 304 and xkey.value == 0:
                    #print('A button was released')
                #elif xkey.code == 305 and xkey.value == 1:
                    #print('B button was pressed')
                #elif xkey.code == 305 and xkey.value == 0:
                    #print('B button was released')
                #elif xkey.code == 306 and xkey.value == 1:
                    #print('??? button was pressed')
                #elif xkey.code == 306 and xkey.value == 0:
                    #print('??? button was released')
                #elif xkey.code == 307 and xkey.value == 1:
                    #print('X button was pressed')
                #elif xkey.code == 307 and xkey.value == 0:
                    #print('X button was pressed')
                elif xkey.code == 308 and xkey.value == 1:
                    print('Y button was pressed - Exiting manual mode.')
                    RunLoop = 'No'
                    break
                elif xkey.code == 308 and xkey.value == 0:
                    print('Y button was released - Exiting manual mode.')
                    RunLoop = 'No'
                    break
                #elif xkey.code == 311 and xkey.value == 1:
                    #print('Right Trigger upper was pressed - Bump car')
                #elif xkey.code == 311 and xkey.value == 0:
                    #print('Right Trigger upper was released')
                #elif xkey.code == 313 and xkey.value == 1:
                    #print('Right Trigger Lower.')
                #elif xkey.code == 313 and xkey.value == 0:
                    #print('Right Trigger lower was released')
                #elif xkey.code == 310 and xkey.value == 1:
                    #print('Left Trigger upper was pressed - Recording video')
                #elif xkey.code == 310 and xkey.value == 0:
                    #print('Left Trigger upper was released')
                #elif xkey.code == 0 and xkey.type == 3:  # Left joystick, left and right.
                    #print('Left joystick - X axis')
                elif xkey.code == 1 and xkey.type == 3:  # Left joystick, Throttle.
                    #print('Left joystick - Y axis - Throttle')
                    #print('value', xkey.value)
                    if xkey.value > 34767: # Backwards
                        #print("backward")
                        GPIO.output(40,True)
                        GPIO.output(in1,GPIO.LOW)
                        GPIO.output(in2,GPIO.HIGH)
                        Thrpwm = (((100-0)/(65535-32767))*xkey.value)+100
                        Thrpwm = max(0,Thrpwm)
                        Thrpwm = min(100,Thrpwm)
                        #print('Throttle pwm : ',Thrpwm)
                        q.ChangeDutyCycle(Thrpwm)
                    elif xkey.value < 30767: # Backwards
                        #print("forward")
                        GPIO.output(40,True)
                        GPIO.output(in1,GPIO.HIGH)
                        GPIO.output(in2,GPIO.LOW)
                        Thrpwm = (((0-100)/(32767-0))*xkey.value)+100
                        Thrpwm = max(0,Thrpwm)
                        Thrpwm = min(100,Thrpwm)
                        #print('Throttle pwm : ',Thrpwm)
                        q.ChangeDutyCycle(Thrpwm)
                    else: # Stop
                        #print("stop")
                        GPIO.output(40,False)
                        GPIO.output(in1,GPIO.LOW)
                        GPIO.output(in2,GPIO.LOW)
                        Thrpwm = 0
                        q.ChangeDutyCycle(Thrpwm)
                elif xkey.code == 2 and xkey.type == 3:  # Right joystick, left and right. # Steering Control
                    #print('----------')
                    #print('Right joystick - X axis')
                    #print('value', xkey.value)
                    Strpwr = (((2.5-12.5)/65535)*xkey.value)+12.5
                    #print('Steering power :',Strpwr)
                    p.ChangeDutyCycle(Strpwr)
                    #print('----------')
                #elif xkey.code == 5 and xkey.type == 3:  # Left joystick, left and right.
                    #print('Right joystick - Y axis')
                #elif xkey.code == 9 and xkey.type == 3:  # Right trigger lower.
                    #print('Right Trigger lower was pressed.')
                    #print('value', xkey.value)
                #elif xkey.code == 10 and xkey.type == 3:  # Left trigger lower.
                    #print('Left Trigger Lower.')
                    #print('value', xkey.value)
                #elif xkey.code == 17 and xkey.value == 0: # Select
                    #print('Select was released')
        else:
            print('-------------------')
            print('Xbox not connected.')
            print('-------------------')
            RunLoop = 'No'
            break

def testheadlights():
    
    print('-------------------')
    print('Testing headlights.')
    print('-------------------')
    print('Ctrl + C to exit.')

    while 1:
        try:
            print('-------------------')
            print('Turn headlights on.')
            print('-------------------')
            GPIO.output(40,True)

            sleep(1)
            
            print('-------------------')
            print('Turn headlights off.')
            print('-------------------')
            GPIO.output(40,False)
            sleep(1)

        except KeyboardInterrupt:
            print('-------------------')
            print('Exiting now. Turning headlights off.')
            print('-------------------')
            GPIO.output(40,False)
            break

if __name__ == "__main__":

    print('-------------------')
    print('Turn on Xbox controller now.')
    print('-------------------')

    print('-------------------')
    print('Starting Main Menu loop.')
    print('-------------------')

    device = 'False'

    while 1:

        inp = int(input('1. Test steering (steps). \
        \n2. Test steering (cycle) \
        \n3. Test rear wheels. \
        \n4. Test camera. \
        \n5. Read Xbox via Bluetooth.\
        \n6. Drive in manual.\
        \n7. Test Headlights.\
        \n8. Exit. \
        \n>>> '))

        if inp == 1:
            TestSteering()
        elif inp == 2:
            TestSteering2()
        elif inp == 3:
            TestRearWheels()
        elif inp == 4:
            getCamera()
        elif inp == 5:
            ReadXboxBT(device)
        elif inp == 6:

            print('-------------------')
            print('Running Control Loop')
            print('-------------------')

            t1 = threading.Thread(target=DriveManualXbox, args=())
            t2 = threading.Thread(target=doCamera, args=())

            t1.start()
            t2.start()

            t1.join()
            t2.join()

            print('Return steering to middle.')
            p.ChangeDutyCycle(7.5)  # turn towards 90 degree

            print("Stopping Motor Driver.")
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.LOW)
            Thrpwm = 0
            q.ChangeDutyCycle(Thrpwm)
            
        elif inp == 7:
            testheadlights()

        elif inp == 8:
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

    print('-------------------')
    print('Turn headlights off.')
    print('-------------------')

    GPIO.output(40,False)

    print('----------------')
    print('Program End.')
    print('----------------')