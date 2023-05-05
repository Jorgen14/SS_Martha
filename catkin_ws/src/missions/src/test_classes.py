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

rate = rospy.Rate(1)   

rospy.loginfo("Starting testing!")

#MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

MarthaCom.clear_waypoints()

if not MarthaCom.is_armed:
    MarthaCom.arm(True)

MarthaCom.change_mode("GUIDED")

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:          
        #MarthaCom.rotate_x_deg(30, -10)
        #MarthaCom.move_forward(0.8)
        MarthaCom.move_sideways(0.8)
        time.sleep(0.2)
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        MarthaCom.stop()
        break

rospy.loginfo("Done!")
