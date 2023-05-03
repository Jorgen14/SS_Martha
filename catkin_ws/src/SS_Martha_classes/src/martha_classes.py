import time
import math
import cv2 as cv
import numpy as np
from ultralytics import YOLO
from datetime import datetime, timedelta
import rospy
from sensor_msgs.msg import NavSatFix, Image, CameraInfo
from std_msgs.msg import Float64
#from std_msgs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import Waypoint, WaypointReached, WaypointList, State, GlobalPositionTarget
from mavros_msgs.srv import WaypointPush, WaypointPull, WaypointClear, CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class droneVision:

    GPS_round = 6
    rel_dist_thresh = 3.0 # Maximum relative distance between buoys
    out_dist = 0.5 # meters to get clear of buoy gate
    wp_dist_from_buoy = 1.0 # meters to set waypoint from yellow buoy
    no_buoy_dist = 2.0 # if no buoys detected move x_meters to look for buoys

    def __init__(self, DEBUG_CAM=False):
        self.DEBUG_CAM = DEBUG_CAM

        self.img_array = None
        self.depth_img = None
        self.wpTimer = 0

        self.communication = apCommunication()
        self.model = YOLO('/home/navo/GitHub/SS_Martha/YOLOv8/buoy_s.pt') 

        self.sub_img = rospy.Subscriber("/zed2/zed_node/left/image_rect_color", Image, self.image_callback)
        self.sub_depth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depth_callback)
        self.sub_cam_info = rospy.Subscriber("/zed2/zed_node/left/camera_info", CameraInfo, self.cam_info_callback)

        try:
            while self.img_array == None:
                rospy.loginfo("Waiting for image...")
                time.sleep(1)
            while self.depth_img == None:
                rospy.loginfo("Waiting for depth image...")
                time.sleep(1)
        except ValueError:
            pass
        
        self.get_detections() # To warm up the model

        rospy.logdebug("Image received, shape: " + str(self.img_array.shape))
        rospy.logdebug("Depth image received, shape: " + str(self.depth_img.shape)) 
        
        rospy.loginfo("Initialized!")

    def image_callback(self, msg):
        img_left = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img_left = img_left[:,:,:3]
        self.img_array = cv.normalize(img_left, None, 0, 255, cv.NORM_MINMAX, cv.CV_8U)

    def depth_callback(self, msg):
        depth_map = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, -1)
        self.depth_img = depth_map[:,:, 0] 

    def cam_info_callback(self, msg):
        width = msg.width
        fx = msg.K[0]
        hfov = np.degrees(2 * np.arctan(width / (2 * fx)))
        self.lamda_x = hfov / width
        self.cx = msg.K[2]
        
    def get_detections(self):
        rospy.logdebug("Getting detections...")
        return self.model.predict(source=self.img_array, conf=0.5, show=self.DEBUG_CAM) 
    
    def detection_results(self):
        results = self.get_detections()
        self.depth_is_nan = False

        rospy.logdebug("Getting detection results...")

        self.color_list = []
        self.depth_list = []
        self.bearing_list = []

        for result in results:
            for box in result.boxes.xyxy:
                center = ((box[2].item() - box[0].item()) / 2 + box[0].item(), 
                          (box[3].item() - box[1].item()) / 2 + box[1].item())
                depth_at_det = self.depth_img[int(center[1]), int(center[0])]
                if math.isnan(depth_at_det):
                    self.depth_is_nan = True
                self.depth_list.append(depth_at_det)
                Tx = int(center[0]) - self.cx
                theta = Tx * self.lamda_x
                self.bearing_list.append(theta)
            for d_cls in result.boxes.cls:
                self.color_list.append(self.model.names[int(d_cls)])

        rospy.logdebug("Colors: " + str(self.color_list))
        rospy.logdebug("Depths: " + str(self.depth_list))
        rospy.logdebug("Bearings: " + str(self.bearing_list))
    
    def get_closest_buoy(self):
        rospy.logdebug("Getting closest buoy...")

        self.closest_color = None
        self.closest_dist = None
        self.closest_bearing = None
        self.closest_is_none = True

        depth_list_sorted = sorted(self.depth_list)
        try:
            self.closest_dist = depth_list_sorted[0]
            closest_index = self.depth_list.index(self.closest_dist)
            self.closest_dist = round(self.closest_dist, 2)
            self.closest_color = self.color_list[closest_index]
            self.closest_bearing = round(self.bearing_list[closest_index], 2)
            self.closest_is_none = False
            rospy.logdebug("Distance to " + self.closest_color + ": " + str(self.closest_dist) + "m " + "at bearing: " + str(self.closest_bearing) + " degrees.")
        except IndexError:
            rospy.logdebug("No buoys detected!")
            pass
    
    def get_2nd_closest_buoy(self):
        rospy.logdebug("Getting second closest buoy...")

        self.second_closest_color = None
        self.second_closest_dist = None
        self.second_closest_bearing = None
        self.second_is_none = True

        depth_list_sorted = sorted(self.depth_list)
        try:
            self.second_closest_dist = depth_list_sorted[1]
            second_closest_index = self.depth_list.index(self.second_closest_dist)
            self.second_closest_dist = round(self.second_closest_dist, 2)
            self.second_closest_color = self.color_list[second_closest_index]
            self.second_closest_bearing = round(self.bearing_list[second_closest_index], 2)
            self.second_is_none = False
            rospy.logdebug("Distance to " + self.second_closest_color + ": " + str(self.second_closest_dist) + "m " + "at bearing: " + str(self.second_closest_bearing) + " degrees.")
        except IndexError:
            rospy.logdebug("Second buoy not detected!")
            pass

    def check_buoy_gate(self):
        if self.closest_color == 'green_buoy' and self.second_closest_color == 'red_buoy':
            rospy.logdebug("Gate found.")
            return True
        elif self.closest_color == 'red_buoy' and self.second_closest_color == 'green_buoy':
            rospy.logdebug("Gate found.")
            return True
        else:
            rospy.logdebug("Gate not found.")
            return False

    def check_gate_orientation(self): # Assumes check_buoy_gate is already called
        try:
            if self.closest_color == "green_buoy":
                if (self.closest_bearing - self.second_closest_bearing) < 0: 
                    rospy.logdebug("Green buoy is on the left, and red buoy is on the right.")
                    return True
                else:
                    rospy.logdebug("Red buoy is on the left, and green buoy is on the right.") 
                    return False
            else:
                if (self.closest_bearing - self.second_closest_bearing) > 0:                     
                   rospy.logdebug("Green buoy is on the left, and red buoy is on the right.")
                   return True                                                                  
                else:                                                                            
                   rospy.logdebug("Red buoy is on the left, and green buoy is on the right.")
                   return False    
        except TypeError:
            rospy.logdebug("Can't check gate orientation!")

    def check_rel_dist(self):
        self.rel_dist_err = False
        try:
            a = self.closest_dist
            c = self.second_closest_dist
            beta = np.radians(self.closest_bearing - self.second_closest_bearing)
            rel_dist = np.sqrt(a**2 + c**2 - 2*a*c*np.cos(beta))
            rospy.logdebug("Relative distance between the two closest buoys: " + str(rel_dist))
        
            if rel_dist < self.rel_dist_thresh:
                rospy.logdebug("Relative distance is less than threshold.")
                return True
            else:
                rospy.logdebug("Relative distance between buoys is greater than threshold.")
                return False
        except TypeError:
            rospy.logerr("Invalid value for distance or bearing")
            self.rel_dist_err = True

    def buoy_GPS_loc(self, R=6371e3):
        self.closest_GPS = []
        self.second_closest_GPS = []

        drone_lat_rad = np.radians(self.communication.lat)
        drone_lon_rad = np.radians(self.communication.lon)
        drone_heading = self.communication.heading

        if not self.closest_is_none:
            closest_bearing_rad = np.radians(drone_heading + self.closest_bearing) # With respect to North
            closest_buoy_lat = np.arcsin(np.sin(drone_lat_rad) * np.cos(self.closest_dist/R) + np.cos(drone_lat_rad) * np.sin(self.closest_dist/R) * np.cos(closest_bearing_rad))
            closest_buoy_lon = drone_lon_rad + np.arctan2(np.sin(closest_bearing_rad) * np.sin(self.closest_dist/R) * np.cos(drone_lat_rad), np.cos(self.closest_dist/R) - np.sin(drone_lat_rad) * np.sin(closest_buoy_lat))
            self.closest_GPS.append(round(np.degrees(closest_buoy_lat), self.GPS_round))
            self.closest_GPS.append(round(np.degrees(closest_buoy_lon), self.GPS_round)) 
            rospy.logdebug("Closest buoy GPS: " + str(self.closest_GPS))

            if not self.second_is_none: # and not self.closest_color == 'yellow_buoy':
                second_closest_bearing_rad = np.radians(drone_heading + self.second_closest_bearing) # With respect to North
                second_closest_buoy_lat = np.arcsin(np.sin(drone_lat_rad) * np.cos(self.second_closest_dist/R) + np.cos(drone_lat_rad) * np.sin(self.second_closest_dist/R) * np.cos(second_closest_bearing_rad))
                second_closest_buoy_lon = drone_lon_rad + np.arctan2(np.sin(second_closest_bearing_rad) * np.sin(self.second_closest_dist/R) * np.cos(drone_lat_rad), np.cos(self.second_closest_dist/R) - np.sin(drone_lat_rad) * np.sin(second_closest_buoy_lat))
                self.second_closest_GPS.append(round(np.degrees(second_closest_buoy_lat), self.GPS_round))
                self.second_closest_GPS.append(round(np.degrees(second_closest_buoy_lon), self.GPS_round)) 
                rospy.logdebug("Second closest buoy GPS: " + str(self.second_closest_GPS))
            #elif self.closest_color == 'yellow_buoy':
             #   rospy.loginfo("Yellow buoy detected.")
            else:
                rospy.logwarn("Only one buoy detected!")

        else:
            rospy.logerr("No buoys detected!")

    @staticmethod
    def dist_to_GPS_cords(dist, bearing, lat, lon, R=6371e3):
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)

        a = np.radians(bearing)

        lat2 = np.arcsin(np.sin(lat_rad) * np.cos(dist/R) + np.cos(lat_rad) * np.sin(dist/R) * np.cos(a))
        lon2 = lon_rad + np.arctan2(np.sin(a) * np.sin(dist/R) * np.cos(lat_rad), np.cos(dist/R) - np.sin(lat_rad) * np.sin(lat2))
        
        lat_deg = round(np.degrees(lat2), 6)
        lon_deg = round(np.degrees(lon2), 6)  

        rospy.logdebug("Waypoint set at: (" + str(lat_deg) + ", " + str(lon_deg) + ")")
        
        return lat_deg, lon_deg
    
    @staticmethod
    def two_points_bearing(lat1, lon1, lat2, lon2):
        lat1_rad = np.radians(lat1)
        lon1_rad = np.radians(lon1)
        lat2_rad = np.radians(lat2)
        lon2_rad = np.radians(lon2)

        x = np.sin(lon2_rad - lon1_rad) * np.cos(lat2_rad)
        y = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(lon2_rad - lon1_rad)
        two_points_bearing = np.degrees(np.arctan2(x, y))
        rospy.logdebug("Bearing between two GPS points: " + str(two_points_bearing))
        return two_points_bearing
    
    @staticmethod
    def hdg_rel_to_bearing(_bearing, rel_angle, drone_hdg):
        out_heading = None
        if (_bearing >= 0 and _bearing < 90) or (_bearing < 0 and _bearing >= -90):
            if (drone_hdg >= 270 and drone_hdg < 360) or (drone_hdg >= 0 and drone_hdg < 90):
                out_heading = _bearing - rel_angle
            else:
                out_heading = _bearing + rel_angle
        elif (_bearing >= 90 and _bearing <= 180) or (_bearing < -90 and _bearing >= -180):
            if drone_hdg >= 90 and drone_hdg < 270:
                out_heading = _bearing + rel_angle
            else:
                out_heading = _bearing - rel_angle

        return out_heading

    def obstacle_channel_gate(self):
        self.wp_lat = round((self.closest_GPS[0] + self.second_closest_GPS[0]) / 2, self.GPS_round)
        self.wp_lon = round((self.closest_GPS[1] + self.second_closest_GPS[1]) / 2, self.GPS_round)

        buoys_bearing = self.two_points_bearing(self.closest_GPS[0], self.closest_GPS[1], self.second_closest_GPS[0], self.second_closest_GPS[1])
        rospy.logdebug("Buoys bearing: " + str(buoys_bearing))
        out_gate_heading = self.hdg_rel_to_bearing(buoys_bearing, 90, self.communication.heading)
        rospy.logdebug("Out of gate heading: " + str(out_gate_heading))
        self.wp_lat_out, self.wp_lon_out = self.dist_to_GPS_cords(self.out_dist, out_gate_heading, self.wp_lat, self.wp_lon)

        rospy.loginfo("Waypoint set at: " + "(" +  str(self.wp_lat) + ", " + str(self.wp_lon) + ")")
        rospy.loginfo("Continuing " + str(self.out_dist) + "m to get clear of gate, to waypoint: " + "(" +  str(self.wp_lat_out) + ", " + str(self.wp_lon_out) + ")")
    
    def obstacle_channel_yellow_buoy(self):
        drone_buoy_bearing = self.two_points_bearing(self.communication.lat, self.communication.lon, self.closest_GPS[0], self.closest_GPS[1])
        
        if self.closest_bearing < 0:
            self.wp_dist_from_buoy *= -1
        else:
            self.wp_dist_from_buoy *= 1
        
        dist_to_wp = np.sqrt(self.closest_dist**2 + self.wp_dist_from_buoy**2)
        angle_buoy_wp = np.degrees(np.arctan2(self.wp_dist_from_buoy, self.closest_dist))
        self.wp_yellow_buoy_lat, self.wp_yellow_buoy_lon = self.dist_to_GPS_cords(dist_to_wp, drone_buoy_bearing + angle_buoy_wp, self.communication.lat, self.communication.lon)

        rospy.loginfo("Navigating around yellow_buoy, waypoint set at: " + "(" +  str(self.wp_yellow_buoy_lat) + ", " + str(self.wp_yellow_buoy_lon) + ")")
    
    def nav_channel_waypoint(self):
        if not self.communication.wp_set:
            self.detection_results()
            if not self.depth_is_nan:
                self.get_closest_buoy() 
                self.get_2nd_closest_buoy()
                self.buoy_GPS_loc()
                if self.check_buoy_gate():
                    if self.check_gate_orientation():
                        self.obstacle_channel_gate()
                        self.communication.change_mode("GUIDED")
                        self.communication.clear_waypoints()
                        self.communication.make_waypoint(self.wp_lat, self.wp_lon, curr=True) 
                        self.communication.make_waypoint(self.wp_lat_out, self.wp_lon_out)
                        self.communication.send_waypoint()
                        self.communication.change_mode("AUTO")
                    else:
                        rospy.loginfo("Buoy gate detected, but not in the right orientation.")
                        rospy.loginfo("Rotating 180 degrees")              

                elif self.closest_color == "yellow_buoy" and self.second_is_none:
                    rospy.loginfo("Yellow buoy detected.")
                    self.obstacle_channel_yellow_buoy()
                    self.communication.change_mode("GUIDED")
                    self.communication.send_guided_wp(self.wp_yellow_buoy_lat, self.wp_yellow_buoy_lon)

                elif (self.closest_color == "red_buoy" or self.closest_color == "green_buoy") and self.second_is_none:
                    rospy.logwarn("Only one non yellow buoy detected, moving closer to get a better look.")
                    self.communication.change_mode("GUIDED")
                    self.communication.send_guided_wp(self.closest_GPS[0], self.closest_GPS[1])

                else:
                    rospy.loginfo("No detections, moving " + str(self.no_buoy_dist) + "m forward to check again.")
                    self.wp_lat, self.wp_lon = self.dist_to_GPS_cords(self.no_buoy_dist, 0, self.communication.lat, self.communication.lon) 
                    self.communication.change_mode("GUIDED")
                    self.communication.send_guided_wp(self.wp_lat, self.wp_lon)

                self.wpTimer = datetime.now() + timedelta(seconds=15)
                time.sleep(1)

            else:
                rospy.logerr("Depth is NaN, trying again...")
            
        elif datetime.now() > self.wpTimer:
            self.communication.wp_set = False
        
        else:
            rospy.loginfo("Waypoints set, waiting " + str(self.wpTimer - datetime.now()) + "s to set next waypoint.")
            time.sleep(1)

# ---------------------------------------------- ROS Communication ---------------------------------------------- #

class apCommunication:
    def __init__(self): 

        self.pub_vel = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.cmd_vel = Twist()
        self.pub_guided_wp = rospy.Publisher("/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)
        self.guided_wp = GlobalPositionTarget()

        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.sub_GPS = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.sub_heading = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.heading_callback)
        self.sub_vel = rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.vel_callback)
        self.sub_wp_reached = rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.wp_reached_callback)
        self.sub_wps = rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self.wps_callback)
        
        self.is_connected = None
        self.is_armed = None
        self.ini_mode = None
        self.ctrl_c = False

        self.arming = None
        self.mode = None
        self.lat = None
        self.lon = None
        self.wp_list = []
        self.wps = None
        self.heading = None
        self.lin_vel_x = None
        self.lin_vel_y = None
        self.ang_vel_z = None
        self.wp_reached = False
        self.reached_seq = None
        self.curr_seq = None
        self.wp_set = False
        self.check = None

        rospy.on_shutdown(self.shutdownhook)

        while (self.is_connected is None) or (not self.is_connected):
            try:
                rospy.loginfo("Waiting for MAVROS connection...")
                time.sleep(1)
            except rospy.ROSInterruptException:
                break

        self.change_mode("LOITER")

        rospy.loginfo("MAVROS connection status: " + str(self.is_connected))
        rospy.loginfo("MAVROS armed status: " + str(self.is_armed))
        rospy.loginfo("MAVROS initial mode: " + str(self.ini_mode))
        time.sleep(1)

    def shutdownhook(self):
        self.ctrl_c = True

    def state_callback(self, data):
        self.is_connected = data.connected
        self.is_armed = data.armed
        self.ini_mode = data.mode

    def arm(self, status):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            if self.arming != status:	
                arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                arming_obj = CommandBoolRequest()
                arming_obj.value = status
                response = arming_client(arming_obj)
                rospy.loginfo("Arming status: " + str(response))
                self.arming = status
            else:
                rospy.logwarn("Vessel is already armed with the desired status!")
        except rospy.ServiceException as e:
            rospy.logerr("Arming failed: %s" %e)

    def change_mode(self, new_mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            if self.mode != new_mode:
                change_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                mode_obj = SetModeRequest()
                mode_obj.custom_mode = new_mode
                response = change_mode_client(mode_obj)
                rospy.loginfo(new_mode + " " + str(response))
                self.mode = new_mode
            else:
                rospy.logwarn("Vessel is already in the desired mode!")
        except rospy.ServiceException as e:
            rospy.logerr("Mode change failed: %s" %e)

    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        rospy.logdebug("Drone GPS location: " + str(self.lat) + ", " + str(self.lon))
    
    def heading_callback(self, msg):
        self.heading = msg.data
        rospy.logdebug("Drone heading: " + str(self.heading))

    def vel_callback(self, msg):
        self.lin_vel_x = msg.twist.linear.x
        self.lin_vel_y = msg.twist.linear.y
        self.ang_vel_z = msg.twist.angular.z
        rospy.logdebug("Drone linear velocity " + "forward: " + str(self.lin_vel_x) + ", " + "sideways: " + str(self.lin_vel_y))
        rospy.logdebug("Drone angular velocity: " + str(self.ang_vel_z))

    def wps_callback(self, msg):
        self.curr_seq = msg.current_seq
        self.wps = msg.waypoints
        rospy.logdebug("Waypoint list: " + str(self.wps))

    def wp_reached_callback(self, msg):
        self.reached_seq = msg.wp_seq
        self.wp_reached = True 
        rospy.logwarn("Waypoint reached!")
        rospy.logdebug("Waypoint reached: " + str(self.wp_reached))

    def auto_wp_reached(self): 
        if self.mode == "AUTO" and (self.curr_seq == self.reached_seq):
            time.sleep(0.1)
            if self.curr_seq == self.reached_seq:
                return True
            else:
                return False
        else:
            return False
    
    def guided_wp_reached(self):
        if self.wp_reached and self.mode == "GUIDED":
            return True
        else:
            return False

    def waypoint_reached(self):
        if self.auto_wp_reached() or self.guided_wp_reached():
            self.wp_reached = False
            return True
        else:
            return False

    def make_waypoint(self, lat, lon, cmd=16, curr=False, autCont=True):
        wp = Waypoint()
        wp.frame = 0 # Global frame
        wp.command = cmd  # 16 = Nav command, 
        wp.is_current = curr
        wp.autocontinue = autCont
        wp.param1 = 0  # HOLD time at WP
        wp.param2 = 0  # Acceptance radius, if inside WP count as reached
        wp.param3 = 0  # Pass Radius. If 0 go through WP
        wp.param4 = 0 # float('nan')  # Yaw, 0 for our USV situation
        wp.x_lat = lat # Latitude
        wp.y_long = lon # Longitude
        wp.z_alt = 0
        self.wp_list.append(wp)

    def send_waypoint(self):
        rospy.wait_for_service('mavros/mission/push')
        try:
            serviceReq = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            serviceRes = serviceReq(start_index=0, waypoints=self.wp_list)
            self.wp_set = serviceRes.success
            if self.wp_set:
                rospy.logdebug("Waypoint successfully pushed")
            else:
                rospy.logerr("FAILURE: PUSHING WP!")		
        except rospy.ServiceException as e:
            rospy.logerr("Failed to send WayPoint: %s\n" %e)
    """
    def pull_waypoint(self):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            response = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
            status = response.success
            self.wps = response.wp_recieved
            if status:
                rospy.loginfo("Waypoints: " + str())
                rospy.logdebug("Waypoint successfully pulled!")
            else:
                rospy.logerr("FAILURE: PULLING WPS!")		
        except rospy.ServiceException as e:
            rospy.logerr("Failed to pull WayPoint: %s\n" %e)
    """
    def clear_waypoints(self):
        rospy.wait_for_service('mavros/mission/clear')
        try:
            response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            self.wp_list = []
            self.make_waypoint(0, 0)		
            return response.call().success
        except rospy.ServiceException as e:
            rospy.logerr("Clear waypoint failed: %s"%e)
            return False        

    def send_guided_wp(self, lat, lon, yaw=0):
        self.guided_wp.latitude = lat
        self.guided_wp.longitude = lon
        self.guided_wp.altitude = 0
        self.guided_wp.yaw = yaw
        while not self.ctrl_c:
            connections = self.pub_guided_wp.get_num_connections()
            if connections > 0:
                self.pub_guided_wp.publish(self.guided_wp)
                rospy.loginfo("GUIDED waypoint published!")
                self.wp_set = True
                break
            else:
                rospy.logdebug("No subscribers to guided_wp yet, waiting to try again!")
                time.sleep(0.1)
        
    def rotate_right(self, ang_vel):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = ang_vel
        self.pub_vel.publish(self.cmd_vel)

    def rotate_left(self, ang_vel):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = -ang_vel
        self.pub_vel.publish(self.cmd_vel)
    
    def move_forward(self, lin_vel):
        self.cmd_vel.linear.x = lin_vel
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        self.pub_vel.publish(self.cmd_vel)
    
    def stop(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        self.pub_vel.publish(self.cmd_vel)
