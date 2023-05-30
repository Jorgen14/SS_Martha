#!/usr/bin/env python3.8
import rospy
import time
import Jetson.GPIO as GPIO
import smbus
from martha_classes import droneVision, apCommunication
from datetime import datetime, timedelta

# Get I2C bus
DEVICE_BUS1 = 1
DEVICE_BUS0 = 0

bus1 = smbus.SMBus(DEVICE_BUS1)
bus0 = smbus.SMBus(DEVICE_BUS0)

# GPIO setup
GPIO.setmode(GPIO.BOARD)
emergency_stop = 15
GPIO.setup(emergency_stop, GPIO.IN)

# I2C address of the device
DEVICE_ADDRESS1 = 0x18
DEVICE_ADDRESS2 = 0x19
DEVICE_ADDRESS3 = 0x18
DEVICE_ADDRESS4 = 0x19

# Relay functions
def relay_on(bus, address):
    bus.write_byte(address, 0x01)

def relay_off(bus, address):
    bus.write_byte(address, 0x00)

ROS_DEBUG = False

if ROS_DEBUG:
    rospy.init_node("NavChannel", log_level=rospy.DEBUG)
else:
    rospy.init_node("NavChannel")  

rate = rospy.Rate(1)   

MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

MarthaCom.clear_waypoints()

if MarthaCom.mode == 'AUTO' or MarthaCom.mode == 'GUIDED':# and not killswitch:
    relay_on(bus1, DEVICE_ADDRESS1)
    relay_off(bus1, DEVICE_ADDRESS2)
    relay_off(bus0, DEVICE_ADDRESS3)
    relay_on(bus0, DEVICE_ADDRESS4)
elif MarthaCom.mode == 'MANUAL':# and not killswitch:
    relay_on(bus1, DEVICE_ADDRESS1)
    relay_off(bus1, DEVICE_ADDRESS2)
    relay_on(bus0, DEVICE_ADDRESS3)
    relay_off(bus0, DEVICE_ADDRESS4)
else:
    relay_on(bus1, DEVICE_ADDRESS1)
    relay_off(bus1, DEVICE_ADDRESS2)
    relay_off(bus0, DEVICE_ADDRESS3)
    relay_on(bus0, DEVICE_ADDRESS4)

if not MarthaCom.is_armed:
    MarthaCom.arm(True)

firstLat = float(input("Latitude: "))
firstLon = float(input("Longitude: "))
MarthaCom.change_mode("GUIDED")
MarthaCom.send_guided_wp(firstLat, firstLon)

while not MarthaCom.waypoint_reached() or not rospy.is_shutdown():# or not killswitch:
    try:
        killswitch = GPIO.input(emergency_stop)
        if (MarthaCom.ini_mode == 'AUTO' or MarthaCom.ini_mode == 'GUIDED') and killswitch:
            relay_on(bus1, DEVICE_ADDRESS1)
            relay_off(bus1, DEVICE_ADDRESS2)
            relay_off(bus0, DEVICE_ADDRESS3)
            relay_on(bus0, DEVICE_ADDRESS4)
        elif MarthaCom.ini_mode == 'MANUAL' and killswitch:
            relay_on(bus1, DEVICE_ADDRESS1)
            relay_off(bus1, DEVICE_ADDRESS2)
            relay_on(bus0, DEVICE_ADDRESS3)
            relay_off(bus0, DEVICE_ADDRESS4)
        elif not killswitch:
            relay_off(bus1, DEVICE_ADDRESS1)
            relay_on(bus1, DEVICE_ADDRESS2)
            relay_off(bus0, DEVICE_ADDRESS3)
            relay_off(bus0, DEVICE_ADDRESS4)
            MarthaCom.change_mode("MANUAL")
            break
        else:
            relay_off(bus1, DEVICE_ADDRESS1)
            relay_off(bus1, DEVICE_ADDRESS2)
            relay_off(bus0, DEVICE_ADDRESS3)
            relay_off(bus0, DEVICE_ADDRESS4)
        rospy.loginfo("On my way to startpoint...")
    except rospy.ROSInterruptException:
        break
"""
while not rospy.is_shutdown():
    startTime = datetime.now()
    try:
        if MarthaCom.mode == 'AUTO' or MarthaCom.mode == 'GUIDED':# and not killswitch:
            relay_on(bus1, DEVICE_ADDRESS1)
            relay_off(bus1, DEVICE_ADDRESS2)
            relay_off(bus0, DEVICE_ADDRESS3)
            relay_on(bus0, DEVICE_ADDRESS4)
        elif MarthaCom.mode == 'MANUAL':# and not killswitch:
            relay_on(bus1, DEVICE_ADDRESS1)
            relay_off(bus1, DEVICE_ADDRESS2)
            relay_on(bus0, DEVICE_ADDRESS3)
            relay_off(bus0, DEVICE_ADDRESS4)
        else:
            relay_on(bus1, DEVICE_ADDRESS1)
            relay_off(bus1, DEVICE_ADDRESS2)
            relay_off(bus0, DEVICE_ADDRESS3)
            relay_on(bus0, DEVICE_ADDRESS4)          
        #MarthaVision.nav_channel_mission()
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        break
"""
MarthaVision.zed.close()

"""
    elif killswitch:
		relay_off(bus1, DEVICE_ADDRESS1)
		relay_on(bus1, DEVICE_ADDRESS2)
		relay_off(bus0, DEVICE_ADDRESS3)
		relay_off(bus0, DEVICE_ADDRESS4)
		MarthaCom.change_mode("MANUAL")
        break
"""