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
        if first:
            MarthaCom.rotate_right()
            if datetime.now() > wpTimer:
                first = False
                second = True
                wpTimer = datetime.now() + timedelta(seconds=10)
        elif second:
            MarthaCom.rotate_left()
            if datetime.now() > wpTimer:
                second = False
                done = True
        elif wpTimer > datetime.now():
            break
        else:
            time.sleep(0.5)
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        break

# while not rospy.is_shutdown():
#     startTime = datetime.now()
#     try:
#         if first:
#             MarthaCom.move_forward()
#             if datetime.now() > wpTimer:
#                 first = False
#                 second = True
#                 wpTimer = datetime.now() + timedelta(seconds=10)
#         elif second:
#             MarthaCom.stop()
#             if datetime.now() > wpTimer:
#                 second = False
#                 done = True
#         elif wpTimer > datetime.now():
#             break
#         else:
#             time.sleep(0.5)
#         scriptTime = datetime.now() - startTime
#         rospy.loginfo("Script time: " + str(scriptTime))
#         rate.sleep()
        
#     except rospy.ROSInterruptException:
#         break