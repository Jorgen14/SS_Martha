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
        """
        if not first:
            MarthaCom.send_guided_wp(-35.36334, 149.16524)
            first = True
        elif MarthaCom.waypoint_reached():
            break

        """        
        if not first:
            MarthaCom.send_guided_wp(-35.36222, 149.16509)
            #MarthaCom.send_guided_wp(-35.36334, 149.16524)
            start_timer = datetime.now() + timedelta(seconds=15)
            first = True
        elif datetime.now() >= start_timer and not second:
            MarthaCom.clear_guided_wp()
            start_timer = datetime.now() + timedelta(seconds=5)
            second = True
        elif datetime.now() >= start_timer and not third:
            #MarthaCom.send_guided_wp(-35.36222, 149.16509)
            MarthaCom.send_guided_wp(-35.36334, 149.16524)
            third = False
        elif datetime.now() >= start_timer and first and second and third: 
            break
        else:
            rospy.loginfo("Waiting...")
            time.sleep(0.2)
        
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        MarthaCom.stop()
        break

rospy.loginfo("Done!")


#MarthaCom.rotate_x_deg(30, -10)
#MarthaCom.move_forward(0.8)
#MarthaCom.move_sideways(0.8)
