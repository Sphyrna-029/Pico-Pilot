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
delay = 0.001  # 1 microsecond
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


#EasyDriver
GPIO.setmode(GPIO.BCM)
GPIO.setup(trackConfig["ms1pin"], GPIO.OUT)
GPIO.setup(trackConfig["ms2pin"], GPIO.OUT)

#Azimuth
GPIO.setup(trackConfig["AziConf"]["AziStepGPIO"], GPIO.OUT)
GPIO.setup(trackConfig["AziConf"]["AziDirGPIO"], GPIO.OUT)

#Elevation
GPIO.setup(trackConfig["AltConf"]["AltStepGPIO"], GPIO.OUT)
GPIO.setup(trackConfig["AltConf"]["AltDirGPIO"], GPIO.OUT)


'''
===============================
 MS1	    MS2	       Mode
===============================
Low	        Low	    Full step
High	    Low	    Half step
Low	        High    Quarter step
High	    High	Eighth step
===============================
'''

def step(steps, dir, microsteps, motorpin, dir_pin):
    count = 0

    if dir == "cw":
        GPIO.output(dir_pin, GPIO.LOW)
    
    elif dir == "cc":
        GPIO.output(dir_pin, GPIO.HIGH)

    if microsteps == 1:
        GPIO.output(trackConfig["ms1pin"], GPIO.LOW)
        GPIO.output(trackConfig["ms2pin"], GPIO.LOW)

    elif microsteps == 2:
        GPIO.output(trackConfig["ms1pin"], GPIO.HIGH)
        GPIO.output(trackConfig["ms2pin"], GPIO.LOW)

    elif microsteps == 4:
        GPIO.output(trackConfig["ms1pin"], GPIO.LOW)
        GPIO.output(trackConfig["ms2pin"], GPIO.HIGH)

    elif microsteps == 8:
        GPIO.output(trackConfig["ms1pin"], GPIO.HIGH)
        GPIO.output(trackConfig["ms2pin"], GPIO.HIGH)

    while count < steps:
        GPIO.output(motorpin, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(motorpin, GPIO.LOW)
        time.sleep(delay)
        count += 1


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
            step(8, "cw", 8, trackConfig["AziConf"]["AziStepGPIO"])

        elif azimuth < inverseDegree(azimuth):
            step(8, "cc", 8, trackConfig["AziConf"]["AziStepGPIO"])
    
        elif rangeCheck(azimuth, target, trackConfig["AziConf"]["AziTolerance"]):
            print("On target! (Azi)")
            return


#Things to know: 200 steps per rotation
#Direction: up(True) or down(False)
#Assume level on power up??? Calibrate on startup
# 50 steps = 90 degrees, 0 = level with horizon, 50 = straight up and down.
#Requirements, no limit switches so we will have to track our steps
#Slew ring is geared, ratio ???
#Allow for manual adjusting on fly

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
            step(8, "cw", 8, trackConfig["AltConf"]["AltStepGPIO"])
            currentStep = currentStep + 1

        elif currentStep > targetAlt:
            step(8, "cw", 8, trackConfig["AltConf"]["AltStepGPIO"])
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
