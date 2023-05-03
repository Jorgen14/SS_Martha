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
go_next = False 
done = False
wpTimer = datetime.now()  

#MarthaVision = droneVision(DEBUG_CAM=False)

MarthaCom = apCommunication()

if not MarthaCom.is_armed:
    MarthaCom.arm(True)

MarthaCom.wp_reached = False

MarthaCom.clear_waypoints()

while not rospy.is_shutdown():
    startTime = datetime.now()
    try:  
        if done:
            #if MarthaCom.mission_completed():
                #rospy.logerr("Mission ended!")
            pass       
        elif not MarthaCom.wp_set:
            MarthaCom.change_mode("GUIDED")
            MarthaCom.send_guided_wp(-35.362240, 149.165058) 
            #MarthaCom.send_guided_wp(-35.363302, 149.165204)      
            #wpTimer = datetime.now() + timedelta(seconds=40)
            rospy.loginfo("First round!")
            """
            MarthaCom.change_mode("GUIDED")
            MarthaCom.clear_waypoints()
            MarthaCom.make_waypoint(-35.362240, 149.165058, curr=True)
            #MarthaCom.make_waypoint(-35.362973, 149.163109)
            #MarthaCom.make_waypoint(-35.363824, 149.163959)
            #MarthaCom.make_waypoint(-35.363302, 149.165204)
            MarthaCom.send_waypoint()
            MarthaCom.change_mode("AUTO")
            """
            time.sleep(1)

        elif MarthaCom.waypoint_reached() and not go_next: #MarthaCom.mission_completed():
            rospy.loginfo("Second round!")
            #MarthaCom.change_mode("GUIDED")
            #MarthaCom.send_guided_wp(-35.363302, 149.165204)
            #MarthaCom.send_guided_wp(-35.362240, 149.165058)
            
            MarthaCom.change_mode("GUIDED")
            MarthaCom.clear_waypoints()
            MarthaCom.make_waypoint(-35.363302, 149.165204, curr=True)
            MarthaCom.make_waypoint(-35.362973, 149.163109)
            #MarthaCom.make_waypoint(-35.363824, 149.163959)
            #MarthaCom.make_waypoint(-35.363302, 149.165204)
            MarthaCom.send_waypoint()
            MarthaCom.change_mode("AUTO")
            time.sleep(1)
            #wpTimer = datetime.now() + timedelta(seconds=20)
            go_next = True 
            #done = True
        elif MarthaCom.waypoint_reached():
            rospy.loginfo("Third round!")
            MarthaCom.change_mode("GUIDED")
            MarthaCom.clear_waypoints()
            MarthaCom.make_waypoint(-35.363824, 149.163959, curr=True)
            MarthaCom.make_waypoint(-35.363302, 149.165204)
            MarthaCom.send_waypoint()
            MarthaCom.change_mode("AUTO")
            time.sleep(1)
        else:
            pass

        rospy.loginfo("Current sequence: " + str(MarthaCom.curr_seq))
        rospy.loginfo("Sequence reached: " + str(MarthaCom.reached_seq))
        rospy.loginfo("Waypoint reached: " + str(MarthaCom.wp_reached))
        scriptTime = datetime.now() - startTime
        rospy.loginfo("Script time: " + str(scriptTime))
        time.sleep(0.5)
        rate.sleep()
        
    except rospy.ROSInterruptException:
        break
