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

rate = rospy.Rate(5)   

first = False
second = False
third = False

start_timer = datetime.now() + timedelta(seconds=125/4)

MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

MarthaCom.clear_waypoints()

if not MarthaCom.is_armed:
    MarthaCom.arm(True)

# firstLat = float(input("Latitude: "))
# firstLon = float(input("Longitude: "))
# MarthaCom.change_mode("GUIDED")
# MarthaCom.send_guided_wp(firstLat, firstLon)

#MarthaCom.change_mode("GUIDED")

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:  
        MarthaVision.detection_results()
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        #MarthaCom.stop()
        break

rospy.loginfo("Done!")


