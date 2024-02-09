from machine import Pin, UART, I2C, PWM
from mpu9250 import MPU9250
from ak8963 import AK8963
from simple_pid import PID
from math import *
import utime, time
import random
import math

class Vehicle(object):
    missionStatus = None
    homeCoordinates = None
    longitude = None
    latitude = None
    speed = None
    altitude = None
    oldaz = None #heh
    azimuth = None
    filtered_magx, filtered_magy = 0, 0

    def __init__(self, picoConfig):
        self.mission = picoConfig["WaypointMission"]["Enabled"]
        self.vconfig = picoConfig

        #PID Rudder
        self.p1 = self.vconfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["P"]
        self.i1 = self.vconfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["I"]
        self.d1 = self.vconfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["D"]

        self.pid1 = PID(self.p1, self.i1, self.d1)

        #PID sample rate
        self.pid1.sample_time = self.vconfig["UpdateFrequencyHz"]
        self.pid1.output_limits = (0, 100)

        #PID Throttle
        self.p2 = self.vconfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["P"]
        self.i2 = self.vconfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["I"]
        self.d2 = self.vconfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["D"]

        self.pid2 = PID(self.p2, self.i2, self.d2)

        #PID sample rate
        self.pid2.sample_time = self.vconfig["UpdateFrequencyHz"]
        self.pid2.output_limits = (0, 100)


    #Stop mission in progress. Returns all vehicle peripherals to neutral. 
    def stop(self):
        self.missionStatus = "stop"

        self.pwm(0, self.vconfig["ChannelMappings"]["Rudder"])
        self.pwm(0, self.vconfig["ChannelMappings"]["Throttle"])


    #Returns distance between two coordinate points in km
    def destinationDistance(self, lon1, lat1, lon2, lat2):
        """
            Calculate the great circle distance between two points 
            on the earth (specified in decimal degrees)
        """
        # convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    
        # haversine formula 
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
    
        # Radius of earth in kilometers is 6371 (Holy cow is there a lot of debate about this)
        km = 6371 * c
    
        return km


    #Low pass filter for mpu2950 mag readings
    def low_pass_filter(self, prev_value, new_value):
        return 0.85 * prev_value + 0.15 * new_value


    #Return human friendly cardinal direction
    def get_cardinal(self, angle):
        if angle > 337 or angle <= 22:
            direction = 'North'

        elif angle > 22 and angle <= 67:
            direction = 'North East'

        elif angle > 67 and angle <= 112:
            direction = "East"

        elif angle > 112 and angle <= 157:
            direction = "South East"

        elif angle > 157 and angle <= 202:
            direction = "South"

        elif angle > 202 and angle <= 247:
            direction = "South West"

        elif angle > 247 and angle <= 292:
            direction = "West"

        elif angle > 292 and angle <= 337:
            direction = "North West"

        return direction


    #Return vehicle azimuth based on true north (Thanks u/QuietRing5299)
    def getAzimuth(self):
        i2c = I2C(0, scl=Pin(1), sda=Pin(0))
        dummy = MPU9250(i2c)#This opens the bybass to access to the AK8963

        ak8963 = AK8963(
            i2c,
            offset=(-10.01602, -19.40039, 50.46211),
            scale=(0.765315, 0.6868211, 4.212917)
        )

        sensor = MPU9250(i2c, ak8963=ak8963)

        magx_new, magy_new, _ = sensor.magnetic
        self.filtered_magx = self.low_pass_filter(self.filtered_magx, magx_new)
        self.filtered_magy = self.low_pass_filter(self.filtered_magy, magy_new)

        heading_angle_in_degrees = math.atan2(self.filtered_magx, self.filtered_magy) * (180 / math.pi)
        heading_angle_in_degrees_plus_declination = heading_angle_in_degrees + float(self.vconfig["Magnometer"]["Declination"])

        if heading_angle_in_degrees_plus_declination < 0:
            heading_angle_in_degrees += 360
            heading_angle_in_degrees_plus_declination += 360

        return heading_angle_in_degrees_plus_declination, self.get_cardinal(heading_angle_in_degrees_plus_declination)


    #Returns vehicle location if available, long first then lat
    def getCoordinates(self):
        if self.longitude:
            if self.latitude:
                return self.longitude, self.latitude

        else:
            return "We lost bro"


    #Take two coordinates and return bearing converted to azimuth
    def getBearing(self, lon1, lat1, lon2, lat2):
        bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
        bearing = degrees(bearing)
        bearing = (bearing + 360) % 360 #Compensated for Azimuth
    
        return bearing


    #Make the pulses
    def pwm(self, val, pinout):
        min = 1000000
        max = 2000000
        pwm = PWM(Pin(pinout))
        pwm.freq(50)
    
        if val > 100 or val < 1:
            stop = max - min / 2 + min #Return servos to middle or stop motors
            pwm.duty_ns(int(stop))
        
            print(pinout)
            print("PWM_Request: Stop") 
            print("DutyCycle: " + str(stop))
        
            return 0, val
    

        else:
            #Convert 1-100 input from PID to a valid pwm duty cycle for our servo/esc
            out = ((( val - 1 ) * (max - min)) / (100 - 1)) + min
            pwm.duty_ns(int(out))
        
            print(pinout)
            print("PWM_Request: " + str(val)) 
            print("DutyCycle: " + str(out))
        
            return out, val


class Telemetry(object):

    def __init__(self, vehicle, picoConfig):
        self.vconfig = picoConfig
        self.vehicle = vehicle

    def dataHandler(self):
        global FIX_STATUS, latitude, longitude, satellites, GPStime, GPSspeedK, GPSspeedN, GPSaltitude
    
        gpsModule = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
        telemetryPort = UART(0, baudrate=self.vconfig["TelemetryBaud"], tx=Pin(12), rx=Pin(13))

        buff = bytearray(255)
        FIX_STATUS = False
        latitude = ""
        longitude = ""
        satellites = ""
        GPStime = ""
        GPSspeedK = ""
        GPSspeedN = ""
        GPSaltitude = ""

        while True:
            gpsModule.readline()
            buff = str(gpsModule.readline())
            parts = buff.split(',')

            if (parts[0] == "b'$GPGGA" and len(parts) == 15):
                if(parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7]):
                    latitude = self.convertToDegree(parts[2])
                
                    if (parts[3] == 'S'):
                      latitude = '-' + latitude
                    
                    longitude = self.convertToDegree(parts[4])
                
                    if (parts[5] == 'W'):
                     longitude = '-' + longitude
                    
                    satellites = parts[7]
                    GPStime = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]

                    #Update our vehicle location
                    self.vehicle.longitude = longitude
                    self.vehicle.latitude = latitude

                    FIX_STATUS = True

            elif (parts[0] == "b'$GPVTG" and len(parts) == 10):
                GPSspeedK = parts[7]
                GPSspeedN = parts[5]

                #self.vehicle.speed = int(GPSspeedK)
                self.vehicle.speed = 3
                
            else:
                FIX_STATUS = False
            
            #Send telemetry data.
            if self.vconfig["Telemetry"]:
    
                dataBuff = []
                #dataBuff.append(getImu())
                dataBuff.append(latitude)
                dataBuff.append(longitude)
                dataBuff.append(satellites)
                dataBuff.append(GPStime)
        
                try:
                    telemetryPort.write(self.databuff)
            
                except:
                    print("Failed to connect to telemetry device")
                
            utime.sleep_ms(500)

    def convertToDegree(self, RawDegrees):
        RawAsFloat = float(RawDegrees)
        firstdigits = int(RawAsFloat/100) 
        nexttwodigits = RawAsFloat - float(firstdigits*100) 
    
        Converted = float(firstdigits + nexttwodigits/60.0)
        Converted = '{0:.6f}'.format(Converted) 

        return str(Converted)
