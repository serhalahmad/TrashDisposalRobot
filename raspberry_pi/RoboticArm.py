# Import libraries
import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Set pin outputs and define
GPIO.setup(16,GPIO.OUT) 
Clipper = GPIO.PWM(16,50) # pin 16 for clipper, pulse 50Hz

GPIO.setup(18,GPIO.OUT) # pin 18 for the arm movement, pulse 50Hz
Arm = GPIO.PWM(18,50)

GPIO.setup(21,GPIO.OUT) # pin 21 for rotation of the base, pulse 50Hz
Base = GPIO.PWM(21,50)

#Functions allow user to use angles to control the servos angle

def openClipper():
    Clipper.start(0)
    angle = 160
    Clipper.ChangeDutyCycle(2+(angle)/18)
    time.sleep(0.5)
    Clipper.ChangeDutyCycle(0)

def closeClipper():
    Clipper.start(0)
    angle = 110
    Clipper.ChangeDutyCycle(2+(angle)/18)
    time.sleep(0.5)
    Clipper.ChangeDutyCycle(0)

def extendArm():
    Arm.start(0)
    angle = 80
    Arm.ChangeDutyCycle(2+(angle)/18)
    time.sleep(2)
    Arm.ChangeDutyCycle(0)

def retractArm():
    Arm.start(0)
    angle = 0
    Arm.ChangeDutyCycle(2+(angle)/18)
    time.sleep(1)
    Arm.ChangeDutyCycle(0)

def disposeTrash():
    extendArm()
    openClipper()
    retractArm()
