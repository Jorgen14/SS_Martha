#!/usr/bin/env python3.8
import rospy
import time
from martha_classes import droneVision, apCommunication
from datetime import datetime, timedelta

ROS_DEBUG = True

if ROS_DEBUG:
    rospy.init_node("TestNode", log_level=rospy.DEBUG)
else:
    rospy.init_node("TestNode")  

rate = rospy.Rate(1)   

first = False
second = False
third = False

start_timer = datetime.now() + timedelta(seconds=125/4)

#MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

MarthaCom.clear_waypoints()

if not MarthaCom.is_armed:
    MarthaCom.arm(True)

MarthaCom.change_mode("GUIDED")

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:  
        MarthaCom.rotate_x_deg(30, 15)
        break
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        MarthaCom.stop()
        break

rospy.loginfo("Done!")
