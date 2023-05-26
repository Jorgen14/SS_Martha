#!/usr/bin/env python3.8
import rospy
import time
from martha_classes import droneVision, apCommunication
from datetime import datetime, timedelta

ROS_DEBUG = False

if ROS_DEBUG:
    rospy.init_node("SpeedGate", log_level=rospy.DEBUG)
else:
    rospy.init_node("SpeedGate")  

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
        MarthaVision.speed_gate_mission()
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        break

MarthaVision.zed.close()
