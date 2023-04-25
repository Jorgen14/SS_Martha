#!/usr/bin/env python3.8
import rospy
import cv2 as cv
from martha_classes import droneVision, droneData
from datetime import datetime

TEST_VISION = False
ROS_DEBUG = True

if ROS_DEBUG:
    rospy.init_node("TestNode", log_level=rospy.DEBUG)
else:
    rospy.init_node("TestNode")                                                 
    
if TEST_VISION:
    MarthaVision = droneVision(DEBUG=True, DEBUG_CAM=False)

MarthaData = droneData(DEBUG=True)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:
        if TEST_VISION:
            MarthaVision.detection_results()
            MarthaVision.get_closest_buoy() 
            MarthaVision.get_2nd_closest_buoy()
            MarthaVision.buoy_GPS_loc(MarthaData.lat, MarthaData.lon, MarthaData.heading)
            MarthaVision.check_buoy_gate()
            if MarthaVision.gate_found:
                MarthaVision.set_waypoint()
            elif MarthaVision.prev_gate_found:
                rospy.loginfo("Previous gate found, turn 180 degrees and look again!")
            else:
                rospy.logwarn("Gate not found!")
        
        else: # TEST_DATA
            pass
       
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()

    except rospy.ROSInterruptException:
        break

if TEST_VISION:
    MarthaVision.zed.close()
