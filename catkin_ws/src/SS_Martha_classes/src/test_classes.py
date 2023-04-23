#!/usr/bin/env python3.8
import rospy
import cv2 as cv
from martha_classes import droneVision, droneData
from datetime import datetime

rospy.init_node("TestNode")

TEST_VISION = True
TEST_DATA = False
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
            if MarthaVision.closest_color == 'yellow_buoy': 
                MarthaVision.buoy_GPS_loc(MarthaData.lat, MarthaData.lon, MarthaData.heading)
                print("Yellow buoy detected at: ", MarthaVision.closest_GPS)
            else:
                MarthaVision.get_2nd_closest_buoy()
                MarthaVision.buoy_GPS_loc(MarthaData.lat, MarthaData.lon, MarthaData.heading)
                if MarthaVision.check_buoy_gate():
                    print("Buoy GPS location: ", MarthaVision.closest_GPS, MarthaVision.second_closest_GPS)
                    MarthaVision.set_waypoint()
                    print("Waypoint: ", MarthaVision.wp_lat, MarthaVision.wp_lon)
                else:
                    if MarthaVision.closest_is_none:
                        print("No buoys detected")
                    else:
                        print("No gate detected")

        if TEST_DATA:
            print("Latitude, longitude, heading: ", MarthaData.lat, MarthaData.lon, MarthaData.heading)
            print("")
            print("Linear speed x and y: ", MarthaData.lin_vel_x, MarthaData.lin_vel_y)
            print("")
            print("Angular speed z: ", MarthaData.ang_vel_z)
            print("")
            print("Waypoint reached: ", MarthaData.wp_reached)
            print("")
        
        print("Script time: ", datetime.now() - startTime)
        rate.sleep()

    except rospy.ROSInterruptException:
        break

MarthaVision.zed.close()
