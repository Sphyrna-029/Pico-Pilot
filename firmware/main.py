from machine import Pin, I2C
import PicoPilot
import _thread
import time
import json

#Open our vehicle config
with open("vehicle.config") as configfile:
    vehicleConfig = json.load(configfile)
    configfile.close()
    print("Config loaded!")

#Instantiate I2c bus
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

telemetry = PicoPilot.Telemetry(vehicleConfig)
vehicle = PicoPilot.Vehicle(vehicleConfig)

#Instantiate dataHandler thread. This thread handles collection of gps data, transmitting telemetry data, and recieving telemetry data.
_thread.start_new_thread(telemetry.dataHandler(), ())

#Assign home position on power up. This will let us enable RTH feature.
home = []

while not home:

    if vehicle.getCoordinates():
        long, lat = vehicle.getCoordinates()
        home.append(long)
        home.append(lat)
        vehicle.homeCoordinates = home
        vehicle.missionStatus = True
        #Do Happy beep

        print("Home saved, mission start!")

    else:
        print("Waiting for home gps fix")

    time.sleep(vehicleConfig["UpdateFrequencyHz"])
        
 
while True:

    if vehicle.getCoordinates():
        vlon, vlat = vehicle.getCoordinates()

        if vehicle.missionStatus:

            for waypoint in vehicleConfig["WaypointMission"]["Waypoints"]:
                waypointDistance = vehicle.destinationDistance(float(vlon), float(vlat), float(waypoint[0]), float(waypoint[1]))
                waypointBearing = vehicle.getBearing(float(vlon), float(vlat), float(waypoint[0]), float(waypoint[1]))

                while waypointDistance <= vehicleConfig["WaypointMission"]["ArrivalThreshold"]:
                    vAzi = vehicle.getAzimuth()
                    vSpeed = vehicle.getSpeed()

                    vehicle.pid1.setpoint = waypointBearing
                    vehicle.pid2.setpoint = vehicleConfig["WaypointMission"]["Speed"]

                    rudderControl = vehicle.pid1(vAzi)
                    throttleControl = vehicle.pid2(vSpeed)

                    rudderPwm, rudderRequest = vehicle.pwm(rudderControl, vehicleConfig["ChannelMappings"]["Rudder"])
                    throttlePwm, throttlRequest = vehicle.pwm(throttlePwm, vehicleConfig["ChannelMappings"]["Throttle"])

                    waypointDistance = vehicle.destinationDistance(vlon, vlat, waypoint[0], waypoint[1])
                    waypointBearing = vehicle.getBearing(vlon, vlat, waypoint[0], waypoint[1])

                    time.sleep(vehicleConfig["UpdateFrequencyHz"])

            #If RTH is enabled, return to home after mission complete.
            if vehicleConfig["WaypointMission"]["ReturnToHome"]:
                waypointDistance = vehicle.destinationDistance(float(vlon), float(vlat), float(vehicle.homeCoordinates[0]), float(vehicle.homeCoordinates[1]))
                waypointBearing = vehicle.getBearing(float(vlon), float(vlat), float(vehicle.homeCoordinates[0]), float(vehicle.homeCoordinates[1]))

                while waypointDistance <= vehicleConfig["WaypointMission"]["ArrivalThreshold"]:
                    vAzi = vehicle.getAzimuth()
                    vSpeed = vehicle.getSpeed()

                    vehicle.pid1.setpoint = waypointBearing
                    vehicle.pid2.setpoint = vehicleConfig["WaypointMission"]["Speed"]

                    rudderControl = vehicle.pid1(vAzi)
                    throttleControl = vehicle.pid2(vSpeed)

                    rudderPwm, rudderRequest = vehicle.pwm(rudderControl, vehicleConfig["ChannelMappings"]["Rudder"])
                    throttlePwm, throttlRequest = vehicle.pwm(throttlePwm, vehicleConfig["ChannelMappings"]["Throttle"])

                    waypointDistance = vehicle.destinationDistance(vlon, vlat, waypoint[0], waypoint[1])
                    waypointBearing = vehicle.getBearing(vlon, vlat, waypoint[0], waypoint[1])

                    time.sleep(vehicleConfig["UpdateFrequencyHz"])
            
            #End mission and return pwm to neutral
            vehicle.stop()

        else:
            print("Waiting for gps fix")

    else:
        print("Misson stopped")
    
    time.sleep(vehicleConfig["UpdateFrequencyHz"])
