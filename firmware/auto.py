from machine import Pin, UART, I2C
from ssd1306 import SSD1306_I2C
import utime, time

#Initialize I2C bus
i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)

#Initialize OLED display
oled = SSD1306_I2C(128, 64, i2c)

#Initialize UART for gps
gpsModule = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
print(gpsModule)

#Initialize UART for telemetry
telemtryModule = UART(0, baudrate=9600, txd=Pin(12), rx=Pin(13))
print(telemtryModule)
