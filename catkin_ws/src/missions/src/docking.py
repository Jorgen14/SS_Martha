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
    rospy.init_node("Docking", log_level=rospy.DEBUG)
else:
    rospy.init_node("Docking")  

rate = rospy.Rate(1)   

MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

MarthaCom.clear_waypoints()

if not MarthaCom.is_armed:
    MarthaCom.arm(True)

firstLat = input("Latitude: ")
firstLon = input("Longitude: ")
MarthaCom.change_mode("GUIDED")
MarthaCom.send_guided_wp(firstLat, firstLon)

while not MarthaCom.waypoint_reached() or not rospy.is_shutdown():
    rospy.loginfo("On my way to startpoint...")

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:          
        MarthaVision.docking_mission()
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        break

MarthaVision.zed.close()