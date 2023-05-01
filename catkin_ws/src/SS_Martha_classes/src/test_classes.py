#!/usr/bin/env python3.8
import rospy
import time
from martha_classes import droneVision, apCommunication
from datetime import datetime, timedelta

ROS_DEBUG = True

if ROS_DEBUG:
    rospy.init_node("TestNode", log_level=rospy.DEBUG)
else:
    rospy.init_node("TestNode")  

rate = rospy.Rate(1)   

MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

MarthaCom.clear_waypoints()

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:          
        if not MarthaCom.wp_set:
            MarthaVision.detection_results()
            if not MarthaVision.depth_is_nan:
                MarthaVision.get_closest_buoy() 
                MarthaVision.get_2nd_closest_buoy()
                MarthaVision.buoy_GPS_loc()
                if MarthaVision.check_buoy_gate():
                    if MarthaVision.check_gate_orientation():
                        MarthaVision.obstacle_channel_gate()  
                        #MarthaVision.communication.send_waypoint(MarthaVision.wp_lat, MarthaVision.wp_lon, curr=True) 
                        #MarthaVision.communication.send_waypoint(MarthaVision.wp_lat_out, MarthaVision.wp_lon_out)
                    else:
                        rospy.loginfo("Buoy gate detected, but not in the right orientation.")
                        rospy.loginfo("Rotating 180 degrees")              

                elif MarthaVision.closest_color == "yellow_buoy" and MarthaVision.second_is_none:
                    rospy.loginfo("Yellow buoy detected.")
                    MarthaVision.obstacle_channel_yellow_buoy()
                    #MarthaVision.communication.send_waypoint(MarthaVision.wp_yellow_buoy_lat, MarthaVision.wp_yellow_buoy_lon, curr=True)

                elif MarthaVision.closest_color == "yellow_buoy" and not MarthaVision.second_is_none:
                    rospy.loginfo("Yellow buoy detected.")
                    if MarthaVision.check_rel_dist() and not MarthaVision.rel_dist_err:
                        rospy.loginfo("Yellow buoy detected, possibly false negative.")
                        rospy.loginfo("Moving closer to get a better look.")
                        MarthaVision.obstacle_channel_gate()
                        #MarthaVision.communication.send_waypoint(MarthaVision.wp_lat, MarthaVision.wp_lon, curr=True)
                        #MarthaVision.communication.send_waypoint(MarthaVision.wp_lat_out, MarthaVision.wp_lon_out)
                    elif not MarthaVision.check_rel_dist() and not MarthaVision.rel_dist_err:
                        rospy.loginfo("Two buoys detected, but not a gate.")
                        rospy.loginfo("Navigating around yellow buoy.")
                        MarthaVision.obstacle_channel_yellow_buoy()
                        #MarthaVision.communication.send_waypoint(MarthaVision.wp_yellow_buoy_lat, MarthaVision.wp_yellow_buoy_lon, curr=True)
                    else:
                        pass

                elif (MarthaVision.closest_color == "red_buoy" or MarthaVision.closest_color == "green_buoy") and MarthaVision.second_is_none:
                    rospy.loginfo("Only one non yellow buoy detected, moving closer to get a better look.")
                    #MarthaVision.communication.send_waypoint(MarthaVision.closest_GPS[0], MarthaVision.closest_GPS[1], curr=True)

                else:
                    rospy.loginfo("No detections, moving " + str(MarthaVision.no_buoy_dist) + "m forward to check again.")
                    MarthaVision.wp_lon, MarthaVision.wp_lat = MarthaVision.dist_to_GPS_cords(MarthaVision.no_buoy_dist, 0, MarthaVision.communication.lat, MarthaVision.communication.lon) 

            else:
                rospy.logerr("Depth is NaN, trying again...")
                
            timerStart = datetime.now()
            time.sleep(1)
            
        elif datetime.now() < timerStart + timedelta(seconds=5):
            MarthaCom.wp_set = False
        
        else:
            rospy.loginfo("Waypoints set, waiting to set next waypoint.")
            time.sleep(1)
        
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        rate.sleep()
        
    except rospy.ROSInterruptException:
        break

