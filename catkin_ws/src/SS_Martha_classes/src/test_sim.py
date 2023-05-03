#!/usr/bin/env python3.8
import rospy
import time
from martha_classes import droneVision, apCommunication
from datetime import datetime, timedelta

ROS_DEBUG = False

if ROS_DEBUG:
    rospy.init_node("TestSimNode", log_level=rospy.DEBUG)
else:
    rospy.init_node("TestSimNode")  

rate = rospy.Rate(1)   

#MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

MarthaCom.clear_waypoints()

if not MarthaCom.is_armed:
    MarthaCom.arm(True)

MarthaCom.change_mode("LOITER")

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:          
        if not MarthaCom.wp_set:
            MarthaCom.change_mode("GUIDED")
            #MarthaCom.send_guided_wp(-35.362240, 149.165058) 
            MarthaCom.send_guided_wp(-35.363105, 149.163350)
        else:
            break
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        break
