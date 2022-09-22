from machine import Pin, UART, I2C
from ssd1306 import SSD1306_I2C
import utime, time
from imu import MPU6050

i2c = I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
gpsModule = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
imu = MPU6050(i2c)

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
    
selfcheck = post()

if not selfcheck:
    oled.fill(0)
    oled.text("POST -", 0, 0)
    oled.text("POST failed.", 0, 30)
    oled.text("Please check", 0, 40)
    oled.text("device.", 0, 50)
    oled.show()
    time.sleep(2)
    

buff = bytearray(255)

TIMEOUT = False
FIX_STATUS = False

latitude = ""
longitude = ""
satellites = ""
GPStime = ""

def getGPS(gpsModule):
    global FIX_STATUS, TIMEOUT, latitude, longitude, satellites, GPStime
    
    timeout = time.time() + 30 
    while True:
        gpsModule.readline()
        buff = str(gpsModule.readline())
        parts = buff.split(',')
    
        if (parts[0] == "b'$GPGGA" and len(parts) == 15):
            if(parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7]):
                print(buff)
                
                latitude = convertToDegree(parts[2])
                if (parts[3] == 'S'):
                    latitude = '-' + latitude
                longitude = convertToDegree(parts[4])
                if (parts[5] == 'W'):
                    longitude = '-' + longitude
                satellites = parts[7]
                GPStime = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]
                FIX_STATUS = True
                break
                
        if (time.time() > timeout):
            TIMEOUT = True
            break
        utime.sleep_ms(500)

def getImu():
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x)
    gy=round(imu.gyro.y)
    gz=round(imu.gyro.z)
    tem=round(imu.temperature,2)
    print("ax",ax,"\t","ay",ay,"\t","az",az,"\t","gx",gx,"\t","gy",gy,"\t","gz",gz,"\t","Temperature",tem,"        ",end="\r")


def convertToDegree(RawDegrees):

    RawAsFloat = float(RawDegrees)
    firstdigits = int(RawAsFloat/100) 
    nexttwodigits = RawAsFloat - float(firstdigits*100) 
    
    Converted = float(firstdigits + nexttwodigits/60.0)
    Converted = '{0:.6f}'.format(Converted) 
    return str(Converted)
    
    
while True:
    
    getGPS(gpsModule)
    
    if(FIX_STATUS == True):
        print("Printing GPS data...")
        print(" ")
        print("Latitude: "+latitude)
        print("Longitude: "+longitude)
        print("Satellites: " +satellites)
        print("Time: "+GPStime)
        print("----------------------")
        print(getImu())
        
        oled.fill(0)
        oled.text("----------------------", 0, 0)
        oled.text("Lat: "+latitude, 0, 20)
        oled.text("Lng: "+longitude, 0, 30)
        oled.text("Satellites: "+satellites, 0, 40)
        oled.text("Time: "+GPStime, 0, 50)
        oled.show()
        
        FIX_STATUS = False
        
    if(TIMEOUT == True):
        oled.fill(0)
        oled.text("Waiting for GPS fix...", 0, 30)
        oled.show()
        print("No GPS data is found.")
        print(getImu())
        TIMEOUT = False
