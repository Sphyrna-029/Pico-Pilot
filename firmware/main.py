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

#Create our vehicle object
vehicle = PicoPilot.Vehicle(vehicleConfig)

#Vehicle location, speed, etc is updated in telemetry thread. Telemetry thread has no helper functions and is hands off. All we have to worry about is driving :)
telemetry = PicoPilot.Telemetry(vehicle, vehicleConfig)

#Instantiate telemetry thread. This thread handles collection of gps data, updating vehicle info, transmitting telemetry data, and recieving telemetry data.
_thread.start_new_thread(telemetry.dataHandler, ())

#Assign home position on power up. This will let us enable RTH feature.
home = []

while not home:
    if vehicle.getCoordinates():
        home.append(vehicle.longitude)
        home.append(vehicle.latitude)
        vehicle.homeCoordinates = home

        if vehicleConfig["WaypointMission"]["Enabled"]:
            vehicle.missionStatus = True

        #Do Happy beep
        print("Home saved, mission start!")

    else:
        print("Waiting for home gps fix")

    time.sleep(vehicleConfig["UpdateFrequencyHz"])

 
while True:
    if vehicle.latitude and vehicle.longitude:
        if vehicle.missionStatus:
            for waypoint in vehicleConfig["WaypointMission"]["Waypoints"]:
                print("New destination:" + str(waypoint))
                waypointDistance = vehicle.destinationDistance(float(vehicle.longitude), float(vehicle.latitude), float(waypoint[0]), float(waypoint[1]))
                waypointBearing = vehicle.getBearing(float(vehicle.longitude), float(vehicle.latitude), float(waypoint[0]), float(waypoint[1]))
    
                print(vehicle.longitude, vehicle.latitude)
                print(waypointDistance)
                print(waypointBearing)
                azi, card = vehicle.getAzimuth()
                print(azi)
                print(card)
                
                while waypointDistance >= vehicleConfig["WaypointMission"]["ArrivalThreshold"]:
                    vehicle.pid1.setpoint = waypointBearing
                    vehicle.pid2.setpoint = vehicleConfig["WaypointMission"]["Speed"]

                    azi, _ = vehicle.getAzimuth()
                    rudderControl = vehicle.pid1(azi)
                    throttleControl = vehicle.pid2(vehicle.speed)

                    rudderPwm, rudderRequest = vehicle.pwm(rudderControl, vehicleConfig["VehicleProfile"]["ChannelMappings"]["Rudder"])
                    throttlePwm, throttlRequest = vehicle.pwm(throttleControl, vehicleConfig["VehicleProfile"]["ChannelMappings"]["Throttle"])

                    waypointDistance = vehicle.destinationDistance(float(vehicle.longitude), float(vehicle.latitude), float(waypoint[0]), float(waypoint[1]))
                    waypointBearing = vehicle.getBearing(float(vehicle.longitude), float(vehicle.latitude), float(waypoint[0]), float(waypoint[1]))

                    time.sleep(vehicleConfig["UpdateFrequencyHz"])

            #If RTH is enabled, return to home after mission complete.
            if vehicleConfig["WaypointMission"]["ReturnToHome"]:
                waypointDistance = vehicle.destinationDistance(float(vehicle.longitude), float(vehicle.latitude), float(vehicle.homeCoordinates[0]), float(vehicle.homeCoordinates[1]))
                waypointBearing = vehicle.getBearing(float(vehicle.longitude), float(vehicle.latitude), float(vehicle.homeCoordinates[0]), float(vehicle.homeCoordinates[1]))
                
                while waypointDistance <= vehicleConfig["WaypointMission"]["ArrivalThreshold"]:
                    vehicle.pid1.setpoint = waypointBearing
                    vehicle.pid2.setpoint = vehicleConfig["WaypointMission"]["Speed"]

                    azi, _ = vehicle.getAzimuth()
                    rudderControl = vehicle.pid1(azi)
                    throttleControl = vehicle.pid2(vehicle.speed)

                    rudderPwm, rudderRequest = vehicle.pwm(rudderControl, vehicleConfig["VehicleProfile"]["ChannelMappings"]["Rudder"])
                    throttlePwm, throttlRequest = vehicle.pwm(throttleControl, vehicleConfig["VehicleProfile"]["ChannelMappings"]["Throttle"])

                    waypointDistance = vehicle.destinationDistance(float(vehicle.longitude), float(vehicle.latitude), float(vehicle.homeCoordinates[0]), float(vehicle.homeCoordinates[1]))
                    waypointBearing = vehicle.getBearing(float(vehicle.longitude), float(vehicle.latitude), float(vehicle.homeCoordinates[0]), float(vehicle.homeCoordinates[1]))

                    time.sleep(vehicleConfig["UpdateFrequencyHz"])
            
            #End mission and return pwm to neutral
            vehicle.stop()
            print("Mission Stopped")
            
        else:
            print("Mission Stopped")

    else:
        print("Waiting For GPS Fix")
    
    time.sleep(vehicleConfig["UpdateFrequencyHz"])
