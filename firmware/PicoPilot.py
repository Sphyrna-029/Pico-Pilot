from machine import Pin, UART, I2C, PWM
from simple_pid import PID
from math import *
import utime, time
import random

class Vehicle(object):
    missionStatus = None
    homeCoordinates = None

    def __init__(self, picoConfig):
        self.vtype = picoConfig["VehicleProfile"]["VehicleType"]
        self.mission = picoConfig["WaypointMission"]["Enabled"]
        self.vconfig = picoConfig

        #PID Rudder
        self.p1 = self.picoConfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["P"]
        self.i1 = self.picoConfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["I"]
        self.d1 = self.picoConfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["D"]

        self.pid1 = PID(self.p1, self.i1, self.d1)

        #PID sample rate
        self.pid1.sample_time = self.picoConfig["UpdateFrequencyHz"]
        self.pid1.output_limits = (0, 100)

        #PID Throttle
        self.p2 = self.picoConfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["P"]
        self.i2 = self.picoConfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["I"]
        self.d2 = self.picoConfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["D"]

        self.pid2 = PID(self.p2, self.i2, self.d2)

        #PID sample rate
        self.pid2.sample_time = self.picoConfig["UpdateFrequencyHz"]
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
    
        # Radius of earth in kilometers is 6371
        km = 6371 * c
    
        return km


    #Return vehicle azimuth based on true north 
    def getAzimuth(self):
        
        return random.randint(0, 360)
    
    
    #Returns vehicle speed, requires unit of measurement (mph/kmh)
    def getSpeed(self, unit):
        
        return random.randint(1, 15)

    #Returns vehicle location if available, long first then lat
    def getCoordinates(self):

        if longitude:
            if latitude:
                return longitude, latitude

        else:
            return None

    #Take two coordinates and return bearing converted to azimuth
    def getBearing(self, lon1, lat1, lon2, lat2):
        bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
        bearing = degrees(bearing)
        bearing = (bearing + 360) % 360
    
        return bearing


    def pwm(self, val, pinout):
        min = 1000000
        max = 2000000
    
        if val > 100 or val < 1:
            stop = max - min / 2 + min #Return servos to middle or stop motors
            pwm = PWM(Pin(pinout))
            pwm.freq(50)
            pwm.duty_ns(int(stop))
        
            print(pinout)
            print("PWM_Request: Stop") 
            print("DutyCycle: " + str(stop))
        
            return 0, val
    

        else:
            #Convert 1-100 input from PID to a valid pwm duty cycle for our servo/esc
            #out = ((max - min) * val) + min
            out = ((( val - 1 ) * (max - min)) / (100 - 1)) + min
    
            pwm = PWM(Pin(pinout))
            pwm.freq(50)
            pwm.duty_ns(int(out))
        
            print("PWM_Request: " + str(val)) 
            print("DutyCycle: " + str(out))
        
            return out, val

class Telemetry(object):

    def __init__(self, picoConfig):
        self.picoConfig = picoConfig

    def dataHandler(self):
        global FIX_STATUS, latitude, longitude, satellites, GPStime, GPSspeed, GPSaltitude
    
        gpsModule = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
        telemetryPort = UART(0, baudrate=self.picoConfig["TelemetryBaud"], tx=Pin(12), rx=Pin(13))

        buff = bytearray(255)
        FIX_STATUS = False
        latitude = ""
        longitude = ""
        satellites = ""
        GPStime = ""
        GPSspeed = ""
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
                    FIX_STATUS = True

            elif (parts[0] == "b'$GPVTG"):
                print(parts)
                
            else:
                FIX_STATUS = False
            
            #Send telemetry data.
            if self.picoConfig["Telemetry"]:
    
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
