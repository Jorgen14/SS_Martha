#!/usr/bin/env python3.8
import rospy
import cv2 as cv
from martha_classes import droneVision, droneData
from datetime import datetime

TEST_VISION = True
ROS_DEBUG = False

if ROS_DEBUG:
    rospy.init_node("TestNode", log_level=rospy.DEBUG)
else:
    rospy.init_node("TestNode")                                                 
    
if TEST_VISION:
    MarthaVision = droneVision(DEBUG=False, DEBUG_CAM=False)

MarthaData = droneData(DEBUG=False)

MarthaData.clear_waypoints()

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:
        if TEST_VISION:
            MarthaVision.detection_results()
            MarthaVision.get_closest_buoy() 
            MarthaVision.get_2nd_closest_buoy()
            MarthaVision.buoy_GPS_loc()
            MarthaVision.check_buoy_gate()
            if MarthaVision.gate_found and not MarthaData.wp_set:
                MarthaVision.set_waypoint()
                MarthaData.send_waypoint()
            elif MarthaVision.prev_gate_found:
                rospy.loginfo("Previous gate found, turn 180 degrees and look again!")
            else:
                if MarthaData.wp_set:
                    rospy.loginfo("Waypoint already set!")
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
