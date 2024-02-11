from machine import Pin, I2C
import PicoPilot
import _thread
import time
import json

#Open our vehicle config
with open("vehicle.config") as configfile:
    vehicleConfig = json.load(configfile)
    configfile.close()
    #print("Config loaded!")

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
        #print("Home saved, mission start!")

    else:
        #print("Waiting for home gps fix")
        pass

    time.sleep(vehicleConfig["UpdateFrequencyHz"])

 
while True:
    if vehicle.latitude and vehicle.longitude:
        if vehicle.missionStatus:
            for waypoint in vehicleConfig["WaypointMission"]["Waypoints"]:
                waypointDistance = vehicle.destinationDistance(float(vehicle.longitude), float(vehicle.latitude), float(waypoint[0]), float(waypoint[1]))
                waypointBearing = vehicle.getBearing(float(vehicle.longitude), float(vehicle.latitude), float(waypoint[0]), float(waypoint[1]))
                azi, card = vehicle.getAzimuth()

                '''
                print("New destination: " + str(waypoint))
                print("Destination distance: " + str(waypointDistance))
                print("Destination bearing: " + str(waypointBearing))
                print("Vehicle Location: " + str(vehicle.longitude) + ", " + str(vehicle.latitude))
                print("Vehicle azimuth: " + str(azi))
                print("Cardinal: " + str(card))
                '''
                
                while waypointDistance >= vehicleConfig["WaypointMission"]["ArrivalThreshold"]:
                    vehicle.pid1.setpoint = waypointBearing
                    vehicle.pid2.setpoint = vehicleConfig["WaypointMission"]["Speed"]

                    azi, _ = vehicle.getAzimuth()
                    azi_direction = vehicle.shortest_path(azi, waypointBearing)
                    rudderControl = vehicle.pid1(azi)
                    throttleControl = vehicle.pid2(vehicle.speed)

                    rudderPwm, rudderRequest = vehicle.rud_pwm(rudderControl, vehicleConfig["VehicleProfile"]["ChannelMappings"]["Rudder"], azi_direction)
                    throttlePwm, throttlRequest = vehicle.throt_pwm(throttleControl, vehicleConfig["VehicleProfile"]["ChannelMappings"]["Throttle"])

                    waypointDistance = vehicle.destinationDistance(float(vehicle.longitude), float(vehicle.latitude), float(waypoint[0]), float(waypoint[1]))
                    waypointBearing = vehicle.getBearing(float(vehicle.longitude), float(vehicle.latitude), float(waypoint[0]), float(waypoint[1]))

                    print("Throttle PWM:", throttlePwm, 
                          "Throttle Requested:", throttlRequest, 
                          "Rudder PWM:", rudderPwm, 
                          "Rudder Requested:", rudderRequest, 
                          "Vehicle azimuth:", azi,
                          "Azimuth Shortest Path:", azi_direction)
                    '''
                    print("Throttle Requested: " + str(throttlRequest))
                    print("Rudder PWM: " + str(rudderPwm))
                    print("Rudder Requested: " + str(rudderRequest))
                    print("Vehicle azimuth: " + str(azi))
                    print("Azimuth Shortest Path: " + str(azi_direction))'''

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

                    rudderPwm, rudderRequest = vehicle.rud_pwm(rudderControl, vehicleConfig["VehicleProfile"]["ChannelMappings"]["Rudder"])
                    throttlePwm, throttlRequest = vehicle.throt_pwm(throttleControl, vehicleConfig["VehicleProfile"]["ChannelMappings"]["Throttle"])

                    waypointDistance = vehicle.destinationDistance(float(vehicle.longitude), float(vehicle.latitude), float(vehicle.homeCoordinates[0]), float(vehicle.homeCoordinates[1]))
                    waypointBearing = vehicle.getBearing(float(vehicle.longitude), float(vehicle.latitude), float(vehicle.homeCoordinates[0]), float(vehicle.homeCoordinates[1]))

                    time.sleep(vehicleConfig["UpdateFrequencyHz"])
            
            #End mission and return pwm to neutral
            vehicle.stop()
            print("Mission Stopped")
            
        else:
            print("Mission Stopped")

    else:
        #print("Waiting For GPS Fix")
        pass
    
    time.sleep(vehicleConfig["UpdateFrequencyHz"])
