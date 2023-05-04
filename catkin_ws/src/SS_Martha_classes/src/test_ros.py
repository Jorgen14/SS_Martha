#!/usr/bin/env python3.8
import rospy
import time
from martha_classes import droneVision, apCommunication
from datetime import datetime, timedelta

ROS_DEBUG = False

if ROS_DEBUG:
    rospy.init_node("TestNode", log_level=rospy.DEBUG)
else:
    rospy.init_node("TestNode") 

#MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

MarthaCom.arm(True)
MarthaCom.change_mode("GUIDED")

MarthaCom.clear_waypoints()

rate = rospy.Rate(1)  
wpTimer = datetime.now() + timedelta(seconds=10) 
first = True
second = False
done = False

if not MarthaCom.is_armed:
    MarthaCom.arm(True)

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:
        MarthaCom.change_mode("GUIDED")
        MarthaCom.rotate_x_deg(90, 30)
        #scriptTime = datetime.now() - startTime
        #rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        break
