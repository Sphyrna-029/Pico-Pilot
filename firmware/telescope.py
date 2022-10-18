#!/usr/bin/python
from os.path import exists
from urllib.request import urlopen
import RPi.GPIO as GPIO, time
import json
import time
import math
import ast
import py_qmc5883l


################################## Load Config ##################################
if not exists("tack-config.json"):
    print("Config track-config.json not found. Make sure this is in same directory as telescope.py")
    exit()


with open("track-config.json") as configfile:
    trackConfig = json.load(configfile)
    configfile.close()
    print("Config loaded!")
################################## End Load Config ##################################

#Init vars
targetData = ""
altStep = 0
aziStep = 0
delay = 0.001  # 1 microsecond


#Get data from stellarium
def getData():
    global targetData

    try:
        stellariumResponse = urlopen("http://localhost:8090/api/main/view?coord=altAz")
    except:
        print("Failed to access stellarium api: http://localhost:8090/api/main/view?coord=altAz")

    targetData = json.loads(stellariumResponse.read())

    print(targetData)

    targetData = ast.literal_eval(targetData["altAz"])

    x, y, z = float(targetData[0]), float(targetData[1]), float(targetData[2])

    #Convert stellarium bullshit view api data to radians
    altitude = math.asin( z )
    azimuth = math.atan2(y, x)

    #Convert from radians to degrees
    azimuth = math.degrees(azimuth)
    altitude = math.degrees(altitude)

    print((azimuth * -1) + 180, altitude)

    return azimuth, altitude


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
    global aziStep
    
    #Target azi converted from degrees to steps
    targetAzi = target * trackConfig["StepsPerDeg"]

    while aziStep != targetAzi:

        if azimuth > inverseDegree(azimuth):
            step(1 * trackConfig["AziConf"]["GearRatio"], "cw", trackConfig["StepMode"], trackConfig["AziConf"]["AziStepGPIO"])

        elif azimuth < inverseDegree(azimuth):
            step(1 * trackConfig["AziConf"]["GearRatio"], "cc", trackConfig["StepMode"], trackConfig["AziConf"]["AziStepGPIO"])
    
        elif rangeCheck(azimuth, target, trackConfig["AziConf"]["AziTolerance"]):
            print("On target! (Azi)")
            return


#Things to know: 1600 steps per rotation (8 micro)
#Assume level on power up??? Calibrate on startup
#400 steps to 90
#Requirements, no limit switches so we will have to track our steps
#Slew ring is geared, ratio ???
#Allow for manual adjusting on fly

#Target degress in hours
def gotoAlt(target):

    global altStep

    #Target altitude converted from degrees to steps
    targetAlt = target * trackConfig["StepsPerDeg"]

    #Check if target is below horizon
    if target < trackConfig["AltConf"]["AltMin"]:
        print("Target Alt is out of bounds.")
        return 
    
    #Check if target is greater than straight up (This should never happen)
    if target > trackConfig["AltConf"]["AltMax"]:
        print("Target Alt is out of bounds.")
        return

    while altStep != targetAlt:

        if altStep < targetAlt:
            step(1 * trackConfig["AltConf"]["GearRatio"], "cw", trackConfig["StepMode"], trackConfig["AltConf"]["AltStepGPIO"])
            altStep = altStep + 1

        elif altStep > targetAlt:
            step(1 * trackConfig["AltConf"]["GearRatio"], "cw", trackConfig["StepMode"], trackConfig["AltConf"]["AltStepGPIO"])
            altStep = altStep - 1 

        elif (targetAlt - trackConfig["AltConf"]["AltTolerance"]) <= altStep <= (targetAlt + trackConfig["AltConf"]["AltTolerance"]):
            print("On target! (Alt)")
            return 


#Placeholder main loop
while True:
    azimuth, altitude = getData()
    gotoAzi(azimuth)
    gotoAlt(altitude)

    time.sleep(0.5)
