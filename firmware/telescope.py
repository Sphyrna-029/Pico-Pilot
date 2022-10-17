#!/usr/bin/python
import RPi.GPIO as GPIO, time
from urllib.request import urlopen
import json
import time
import py_qmc5883l


#Load Config
with open("track-config.json") as configfile:
    trackConfig = json.load(configfile)
    configfile.close()
    print("Config loaded!")


#Init vars
targetData = ""
currentStep = 0
#Initialize E-compass
sensor = py_qmc5883l.QMC5883L()
#Set Declination https://www.nwcg.gov/course/ffm/location/65-declination
sensor.declination = trackConfig["CompassConf"]["Declination"]
#Get our Azimuth(bearing from true north)
azimuth = sensor.get_bearing()


#Get data from stellarium
def getData():
    global targetData
    try:
        stellariumResponse = urlopen(trackConfig["stellariumAPI"])
    except:
        print("Failed to access stellarium api: " + trackConfig["stellariumAPI"])

    targetData = json.loads(stellariumResponse.read())

    print(targetData)

    return targetData

#Azimuth
GPIO.setmode(GPIO.BOARD)
GPIO.setup(trackConfig["AziConf"]["AziStepGPIO"], GPIO.OUT)
GPIO.setup(trackConfig["AziConf"]["AziDirGPIO"], GPIO.OUT)
GPIO.setwarnings(False)
GPIO.output(trackConfig["AziConf"]["AziStepGPIO"], True)

#Elevation
GPIO.setmode(GPIO.BOARD)
GPIO.setup(trackConfig["AltConf"]["AltStepGPIO"], GPIO.OUT)
GPIO.setup(trackConfig["AltConf"]["AltDirGPIO"], GPIO.OUT)
GPIO.setwarnings(False)
GPIO.output(trackConfig["AltConf"]["AltStepGPIO"], True)


#PWM setup
aziPWM = GPIO.PWM(trackConfig["AziConf"]["AziStepGPIO"], 5000)
elePWM = GPIO.PWM(trackConfig["AltConf"]["AltStepGPIO"], 5000)


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


#Target degress in hours
def gotoAzi(target):

    while azimuth != target:

        azimuth = sensor.get_bearing()

        if azimuth > inverseDegree(azimuth):
            aziMotor(True, 1)

        elif azimuth < inverseDegree(azimuth):
            aziMotor(False, 1)
    
        elif rangeCheck(azimuth, target, trackConfig["AziConf"]["AziTolerance"]):
            print("On target! (Azi)")
            return


#Things to know: 200 steps per rotation
#Direction: up(True) or down(False)
#Assume level on power up??? Calibrate on startup
# 50 steps = 90 degrees, 0 = level with horizon, 50 = straight up and down.
#Requirements, no limit switches so we will have to track our steps
#Slew ring is geared, ratio ???
#Figure out microstepping
# 90 / 50 = 1.8 steps per degree

#Target degress in hours
def gotoAlt(target):

    global currentStep

    #Target altitude converted from degrees to steps
    targetAlt = target * trackConfig["StepsPerDeg"]

    #Check if target is below horizon
    if target < trackConfig["AltConf"]["AltMin"]:
        print("Target Alt is out of bounds.")
        return 
    
    #Check if target is greater than straight up
    if target > trackConfig["AltConf"]["AltMax"]:
        print("Target Alt is out of bounds.")
        return

    while currentStep != targetAlt:

        if currentStep < targetAlt:
            altMotor(True, 1)
            currentStep = currentStep + 1

        elif currentStep > targetAlt:
            altMotor(False, 1)
            currentStep = currentStep - 1 

        elif (targetAlt - trackConfig["AltConf"]["AltTolerance"]) <= currentStep <= (targetAlt + trackConfig["AltConf"]["AltTolerance"]):
            print("On target! (Alt)")
            return 


#Placeholder main loop
while True:
    getData()
    gotoAzi(targetData["azimuth"])
    gotoAlt(targetData["altitude"])

    time.sleep(1)
