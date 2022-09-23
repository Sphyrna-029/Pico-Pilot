from math import *
from machine import Pin, UART, I2C
from ssd1306 import SSD1306_I2C
from imu import MPU6050
from simple_pid import PID
import utime, time
import _thread
import json
 

#Open our vehicle config
with open("vehicle.config") as configfile:
    vehicleConfig = json.load(configfile)
    configfile.close()
    print("Config loaded!")

#Instantiate our static objects
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
gpsModule = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
telemetryPort = UART(0, baudrate=vehicleConfig["TelemetryBaud"], tx=Pin(12), rx=Pin(13))
imu = MPU6050(i2c)

#Perform self health check
def post():
    oled.fill(0)
    oled.text("POST", 0, 0)
    oled.show()
    time.sleep(2)
   
    devices = i2c.scan()
    
    if len(devices) == 0:
        oled.fill(0)
        oled.text("POST -", 0, 0)
        oled.text("I2c bus error.", 0, 30) 
        oled.show()
        return False
    
    else:
        oled.fill(0)
        oled.text("POST +", 0, 0)
        oled.text("I2c check pass!", 0, 30)
        oled.text(str(devices), 0, 40)
        oled.show()
        time.sleep(5)
        return True

#Determine if POST is required
if vehicleConfig["PowerOnSelfTest"]:
    selfcheck = post()

    if not selfcheck:
        oled.fill(0)
        oled.text("POST -", 0, 0)
        oled.text("POST failed.", 0, 30)
        oled.text("Please check", 0, 40)
        oled.text("device.", 0, 50)
        oled.show()
        time.sleep(2)
    

#GPS Vars
buff = bytearray(255)
FIX_STATUS = False
latitude = ""
longitude = ""
satellites = ""
GPStime = ""


#GPS Handler Function
def dataHandler():
    global FIX_STATUS, latitude, longitude, satellites, GPStime
    
    while True:
        gpsModule.readline()
        buff = str(gpsModule.readline())
        parts = buff.split(',')

        if (parts[0] == "b'$GPGGA" and len(parts) == 15):
            if(parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7]):
                latitude = convertToDegree(parts[2])
                
                if (parts[3] == 'S'):
                    latitude = '-' + latitude
                    
                longitude = convertToDegree(parts[4])
                
                if (parts[5] == 'W'):
                    longitude = '-' + longitude
                    
                satellites = parts[7]
                GPStime = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]
                FIX_STATUS = True
                
        else:
            FIX_STATUS = False
            
        #Send telemetry data.
        if vehicleConfig["Telemetry"]:
    
            dataBuff = []
            dataBuff.append(getImu())
            dataBuff.append(latitude)
            dataBuff.append(longitude)
            dataBuff.append(satellites)
            dataBuff.append(GPStime)
        
            try:
                telemetryPort.write(databuff)
            
            except:
                print("Failed to connect to telemetry device")
                
        utime.sleep_ms(500)

#Gather IMU Data 
def getImu():
    data = []
    
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x)
    gy=round(imu.gyro.y)
    gz=round(imu.gyro.z)
    tem=round(imu.temperature,2)
    
    data.append(ax)
    data.append(ay)
    data.append(az)
    data.append(gx)
    data.append(gy)
    data.append(gz)
    data.append(tem)

    return data

#+++++++++++++++++++++ Helper functions +++++++++++++++++++++

def convertToDegree(RawDegrees):
    RawAsFloat = float(RawDegrees)
    firstdigits = int(RawAsFloat/100) 
    nexttwodigits = RawAsFloat - float(firstdigits*100) 
    
    Converted = float(firstdigits + nexttwodigits/60.0)
    Converted = '{0:.6f}'.format(Converted) 
    return str(Converted)
    
def haversine(lon1, lat1, lon2, lat2):
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

def bearing(lon1, lat1, lon2, lat2):
    bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
    bearing = degrees(bearing)
    bearing = (bearing + 360) % 360
    
    return bearing
    

def vBearing():
    #Code to get vehicle bearing based on true north. Magnometer code goes here. Returns 0-360 degrees
    return 0
    
#+++++++++++++++++++++ End Helper functions +++++++++++++++++++++

#Instaniate dataHandler thread. This thread handles collection of gps data, transmitting telemetry data, and recieving telemetry data.
_thread.start_new_thread(dataHandler, ())

##+++++++++++++++++++++ PID Controllers +++++++++++++++++++++
#PID Rudder
p1 = vehicleConfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["P"]
i1 = vehicleConfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["I"]
d1 = vehicleConfig["VehicleProfile"]["ChannelMappings"]["SteeringPID"]["D"]

pid1 = PID(p1, i1, d1)

#PID sample rate
pid1.sample_time = vehicleConfig["UpdateFrequencyHz"] 

#PID Throttle
p2 = vehicleConfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["P"]
i2 = vehicleConfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["I"]
d2 = vehicleConfig["VehicleProfile"]["ChannelMappings"]["ThrottlePID"]["D"]

pid2 = PID(p2, i2, d2)

#PID sample rate
pid2.sample_time = vehicleConfig["UpdateFrequencyHz"]

##+++++++++++++++++++++ End PID Controllers +++++++++++++++++++++

#Main thread
while True: 
            
    #Get distance to next waypoint
    if vehicleConfig["WaypointMission"]["Enabled"]:
        if latitude:
            print("Calculated distance to waypoint:")
            print(haversine(float(longitude), float(latitude), float(vehicleConfig["WaypointMission"]["Waypoints"][1]), float(vehicleConfig["WaypointMission"]["Waypoints"][0])))
            
            #Get bearing to next waypoint
            print("Bearing to next waypoint:")
            print(bearing(float(longitude), float(latitude), float(vehicleConfig["WaypointMission"]["Waypoints"][1]), float(vehicleConfig["WaypointMission"]["Waypoints"][0])))
            
            
    print("Latitude: " + latitude)
    print("Longitude: " + longitude)
    print("Satellites: " + satellites)
    print("Time: " + GPStime)
    print("----------------------")
    print(getImu())
        
    oled.fill(0)
    oled.text("----------------------", 0, 0)
    oled.text("Lat: "+latitude, 0, 20)
    oled.text("Lon: "+longitude, 0, 30)
    oled.text("Sat: "+satellites, 0, 40)
    oled.text("Tim: "+GPStime, 0, 50)
    oled.show()
        
    time.sleep(vehicleConfig["UpdateFrequencyHz"])
