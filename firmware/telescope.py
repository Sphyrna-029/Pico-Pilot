#!/usr/bin/python
import RPi.GPIO as GPIO, time
import py_qmc5883l


#Azimuth
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setwarnings(False)
GPIO.output(16, True)

#Elevation
GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setwarnings(False)
GPIO.output(13, True)


#PWM setup
aziPWM = GPIO.PWM(16, 5000)
elePWM = GPIO.PWM(13, 5000)


def aziMotor(direction, num_steps):
    aziPWM.ChangeFrequency(5000)
    GPIO.output(18, direction)
    while num_steps > 0:
        aziPWM.start(1)
        time.sleep(0.01)
        num_steps -= 1
    aziPWM.stop()
    GPIO.cleanup()
    return True


def altMotor(direction, num_steps):
    elePWM.ChangeFrequency(5000)
    GPIO.output(18, direction)
    while num_steps > 0:
        elePWM.start(1)
        time.sleep(0.01)
        num_steps -= 1
    elePWM.stop()
    GPIO.cleanup()
    return True


#Return the opposite of our current bearing. This lets us determine the fastest path to our destination
def inverseDegree(x):

    return (x + 180) % 360 


#Check if our azimuth is within a degree of tolerance of our target
def rangeCheck(current, target, tolerance):

    low = (target - tolerance) % 360
    high = (target + tolerance) % 360

    return (current - low) % 360 <= (high - low) % 360



#Initialize E-Compass
sensor = py_qmc5883l.QMC5883L()

#Set Declination https://www.nwcg.gov/course/ffm/location/65-declination
sensor.declination = 10.02

#Get our Azimuth(bearing from true north)
azimuth = sensor.get_bearing()

#Target degress in hours
def gotoAzi(target):

    while azimuth != target:

        azimuth = sensor.get_bearing()

        if azimuth > inverseDegree(azimuth):
            aziMotor(True, 1)

        elif azimuth < inverseDegree(azimuth):
            aziMotor(False, 1)
    
        elif rangeCheck(azimuth, target, 0.5):
            print("On target! (Azi)")
            return


#Things to know: 200 steps per rotation
#Direction: up(True) or down(False)
#Assume level on power up??? Calibrate on startup
# 50 steps = 90 degrees, 0 = level with horizon, 50 = straight up and down.
#Requirements, no limit switches so we will have to track our steps
#Slew ring is geared, ratio ???
# 90 / 50 = 1.8 steps per degree
currentStep = 0

#Target degress in hours
def gotoAlt(target):

    global currentStep

    #Target altitude converted from degrees to steps
    targetAlt = target * 1.8

    if target < 0:
        print("Target Alt is out of bounds.")
        return 
    
    if target > 90:
        print("Target Alt is out of bounds.")
        return

    while currentStep != targetAlt:

        if currentStep < targetAlt:
            altMotor(True, 1)
            currentStep = currentStep + 1

        elif currentStep > targetAlt:
            altMotor(False, 1)
            currentStep = currentStep - 1 

        elif (targetAlt - 1) <= currentStep <= (targetAlt + 1):
            print("On target! (Alt)")
            return 
