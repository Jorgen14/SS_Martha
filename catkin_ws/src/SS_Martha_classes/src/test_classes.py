#!/usr/bin/env python3.8
import rospy
import cv2 as cv
from martha_classes import droneVision, droneData
from datetime import datetime

rospy.init_node("TestNode")

TEST_VISION = True
rate = rospy.Rate(1)

if TEST_VISION:
    MarthaVision = droneVision(DEBUG=True)

MarthaData = droneData(DEBUG=True)

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
                rospy.logerr("Gate not found!")
        
        else: # TEST_DATA
            pass
        
        rospy.loginfo("Script time: ", datetime.now() - startTime)
        rate.sleep()

    except rospy.ROSInterruptException:
        break

if TEST_VISION:
    MarthaVision.zed.close()
