import time
import math
import cv2 as cv
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO
from datetime import datetime, timedelta
import rospy
from sensor_msgs.msg import NavSatFix, Image, CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import Waypoint, WaypointReached, WaypointList, State, GlobalPositionTarget
from mavros_msgs.srv import WaypointPush, WaypointClear, CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class droneVision:

    GPS_round = 6
    rel_dist_thresh = 3.0 # Maximum relative distance between buoys
    out_dist = 0.5 # meters to get clear of buoy gate
    wp_dist_from_buoy = 1.5 # meters to set waypoint from yellow buoy
    no_buoy_dist = 2.0 # if no buoys detected move x_meters to look for buoys
    sg_no_buoy_dist = 5.0 # Same as above but for speed gate
    search_max_iter = 3 # maximum number of times to search for buoys
    look_degs = 45 # Angle for looking to both sides in degrees

    def __init__(self, DEBUG_CAM=False):
        self.DEBUG_CAM = DEBUG_CAM

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD2K # Options: HD2K, HD1080, HD720, VGA
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.sdk_verbose = True

        self.runtime_params = sl.RuntimeParameters()
        self.zed_status = self.zed.open(init_params)
        
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, -1) # -1 = auto
        rospy.loginfo("Exposure: " + str(self.zed.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE)))

        if self.zed_status != sl.ERROR_CODE.SUCCESS:
            print (repr(self.zed_status))
            exit(1)

        positional_tracking_param = sl.PositionalTrackingParameters()
        positional_tracking_param.set_floor_as_origin = True
        self.zed.enable_positional_tracking(positional_tracking_param)

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True
        obj_param.enable_mask_output = True
        self.zed.enable_object_detection(obj_param)

        self.objects = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters() 
        
        positional_tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)

        self.width = self.zed.get_camera_information().camera_resolution.width
        self.hfov = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.h_fov
        self.cx = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.cx
        self.lamda_x = self.hfov / self.width

        self.img_left = sl.Mat()
        self.depth_map = sl.Mat()

        self.wp_lat_out = None
        self.wp_lon_out = None
        self.start_lat = None
        self.start_lon = None

        self.docking_hdg = None
        self.start_docking = False
        self.first_timer = False
        self.docking_timer = None
        self.second_timer = False

        self.yellow_set = False
        self.gate_set = False
        self.mission_complete = False
        
        self.start_time = datetime.now() + timedelta(seconds=60*60)
        self.stb_clear = False
        self.port_clear = False
        self.search_iter = 0

        self.communication = apCommunication()
        self.model = YOLO('/home/navo/GitHub/SS_Martha/YOLOv8/buoy_m1.pt') 
        
        rospy.loginfo("Warming up model...")
        self._image_and_depth_map()
        self._get_detections()

        rospy.logdebug("Image received, shape: " + str(self.np_img_left.shape))
        rospy.logdebug("Depth image received, shape: " + str(self.depth_img.shape)) 
        
        rospy.loginfo("Camera initialized!")
        time.sleep(2)

    def nav_channel_mission(self):
        if not self.communication.wp_set:
            self.detection_results()
            if not self.depth_is_nan:
                self.get_closest_buoy() 
                self.get_2nd_closest_buoy()
                self.buoy_GPS_loc()
                if self.check_buoy_gate():
                    if self.check_gate_orientation():
                        self.gate_wp()
                        self.communication.change_mode("GUIDED")
                        self.communication.clear_waypoints()
                        self.communication.make_waypoint(self.wp_lat, self.wp_lon, curr=True) 
                        self.communication.make_waypoint(self.wp_lat_out, self.wp_lon_out)
                        self.communication.send_waypoint()
                        self.communication.change_mode("AUTO")
                        self.start_timer(self.closest_dist / self.communication.max_speed + 7.5)
                        self.reset_search_vars()
                    else:
                        rospy.loginfo("Buoy gate detected, but not in the right orientation.")
                        self.communication.change_mode("GUIDED")
                        self.communication.rotate_x_deg(180, 60)    
                        self.reset_search_vars()

                elif self.closest_color == "yellow_buoy" and self.second_is_none:
                    self.obstacle_channel_yellow_buoy()
                    self.communication.change_mode("GUIDED")
                    self.communication.send_guided_wp(self.wp_yellow_buoy_lat, self.wp_yellow_buoy_lon)
                    self.start_timer(self.closest_dist / self.communication.max_speed + 5)
                    self.reset_search_vars()

                elif self.closest_color == "yellow_buoy":
                    rospy.loginfo("Yellow buoy detected, but also one other buoy.")
                    if self.check_rel_dist() and not self.rel_dist_err:
                        rospy.loginfo("Yellow buoy detected, but suspiciously close to other buoy.")
                        self.communication.change_mode("GUIDED")
                        self.communication.send_guided_wp(self.closest_GPS[0], self.closest_GPS[1])
                        self.start_timer((self.closest_dist / self.communication.max_speed) / 2)
                        self.reset_search_vars()
                    else:
                        self.obstacle_channel_yellow_buoy()
                        self.communication.change_mode("GUIDED")
                        self.communication.send_guided_wp(self.wp_yellow_buoy_lat, self.wp_yellow_buoy_lon)
                        self.start_timer(self.closest_dist / self.communication.max_speed + 5)
                        self.reset_search_vars()

                elif (self.closest_color == "red_buoy" or self.closest_color == "green_buoy") and self.second_is_none:
                    rospy.logwarn("Only one non yellow buoy detected, moving closer to get a better look.")
                    self.communication.change_mode("GUIDED")
                    self.communication.send_guided_wp(self.closest_GPS[0], self.closest_GPS[1])
                    self.start_timer((self.closest_dist / self.communication.max_speed) / 2)
                    self.reset_search_vars()

                else:
                    if not self.stb_clear:
                        rospy.loginfo("No detections, turning to starboard to check again.")
                        self.communication.change_mode("GUIDED")
                        self.communication.rotate_x_deg(self.look_degs, 20)
                        self.stb_clear = True
                    elif not self.port_clear:
                        rospy.loginfo("No detections, turning to port to check again.")
                        self.communication.change_mode("GUIDED")
                        self.communication.rotate_x_deg(-2*self.look_degs, -20)
                        self.port_clear = True
                    elif self.im_lost():
                        rospy.logerr("I'm lost :(")
                        if not self.wp_lat_out is None and not self.wp_lon_out is None:
                            rospy.logwarn("Returning to last gate exit.")
                            self.communication.change_mode("GUIDED")
                            self.communication.send_guided_wp(self.wp_lat_out, self.wp_lon_out)
                            self.reset_search_vars()
                        else:
                            rospy.logerr("I don't know where I am, pls help.")
                            self.communication.change_mode("GUIDED")
                            self.communication.rotate_x_deg(90, 45)
                    else:
                        rospy.logwarn("No detections, moving " + str(self.no_buoy_dist) + "m forward to check again.")
                        self.wp_lat, self.wp_lon = self.dist_to_GPS_cords(self.no_buoy_dist, self.communication.heading, self.communication.lat, self.communication.lon) 
                        self.communication.change_mode("GUIDED")
                        self.communication.send_guided_wp(self.wp_lat, self.wp_lon)
                        self.stb_clear = False
                        self.port_clear = False
                        self.search_iter += 1

            else:
                rospy.logerr("Depth is NaN, trying again...")
            
        elif self.communication.waypoint_reached():
            self.communication.wp_set = False

        elif self.timer_reached():
            rospy.loginfo("Timer reached, looking again.")
            self.communication.clear_guided_wp()
            self.communication.clear_waypoints()
            self.communication.wp_set = False
        
        else:
            rospy.loginfo("Waypoints set, waiting to reach waypoint before setting next waypoint.")
            time.sleep(0.1)

    def speed_gate_mission(self):
        if self.gate_set and not self.yellow_set and not self.communication.wp_set:
            self.detection_results()
            if not self.depth_is_nan:
                self.get_closest_buoy() 
                self.buoy_GPS_loc()
                if self.closest_color == "yellow_buoy":
                    self.speed_gate_yellow_buoy()
                    self.communication.change_mode("GUIDED")
                    self.communication.clear_waypoints()
                    self.communication.make_waypoint(self.yellow_buoy_lat_1, self.yellow_buoy_lon_1, curr=True) 
                    self.communication.make_waypoint(self.yellow_buoy_lat_2, self.yellow_buoy_lon_2)
                    self.communication.make_waypoint(self.yellow_buoy_lat_3, self.yellow_buoy_lon_3)
                    self.communication.make_waypoint(self.start_lat, self.start_lon)
                    self.communication.make_waypoint(self.start_lat_out_ret, self.start_lon_out_ret)
                    self.communication.send_waypoint()
                    self.communication.change_mode("AUTO")
                    self.yellow_set = True
                else:
                    self.wp_lat, self.wp_lon = self.dist_to_GPS_cords(self.sg_no_buoy_dist, self.communication.heading, self.communication.lat, self.communication.lon) 
                    self.communication.change_mode("GUIDED")
                    self.communication.send_guided_wp(self.wp_lat, self.wp_lon)

            else:
                rospy.logerr("Depth is NaN, trying again...")
            
        elif not self.gate_set:
            self.detection_results()
            if not self.depth_is_nan:
                self.get_closest_buoy() 
                self.get_2nd_closest_buoy()
                self.buoy_GPS_loc()
                try:
                    if self.check_buoy_gate():
                        self.gate_wp()
                        self.communication.change_mode("GUIDED")
                        self.communication.clear_waypoints()
                        self.communication.make_waypoint(self.wp_lat, self.wp_lon, curr=True) 
                        self.communication.make_waypoint(self.wp_lat_out, self.wp_lon_out)
                        self.communication.send_waypoint()
                        self.communication.change_mode("AUTO")
                        self.start_lat = self.wp_lat
                        self.start_lon = self.wp_lon
                        self.start_lat_out_ret = self.wp_lat_out_ret
                        self.start_lon_out_ret = self.wp_lon_out_ret
                        self.gate_set = True
                    else:
                        rospy.logwarn("No gate detected.")
                except IndexError:
                    pass
            else:
                rospy.logerr("Depth is NaN, trying again...")

        elif self.communication.waypoint_reached() and self.yellow_set:
            rospy.loginfo("Mission complete, returning home.")
            self.communication.RTL()
            
        elif self.communication.waypoint_reached():
            self.communication.wp_set = self.start_docking
        
        else:
            rospy.loginfo("Waypoints set, waiting to reach waypoint before setting next waypoint.")
            time.sleep(0.2)

    def docking_mission(self):
        if not self.communication.wp_set:
            self.detection_results()
            if not self.depth_is_nan:
                self.get_closest_buoy() 
                self.get_2nd_closest_buoy()
                self.buoy_GPS_loc()
                try:
                    self.gate_wp()
                    self.communication.change_mode("GUIDED")
                    self.communication.send_guided_wp(self.wp_lat_out_ret, self.wp_lon_out_ret)
                    self.docking_hdg = self.buoys_bearing
                except IndexError:
                    pass
            else:
                rospy.logerr("Depth is NaN, trying again...")

        elif self.timer_reached() and self.docking_timer:
            rospy.loginfo("Docking timer reached, starting undocking...")
            self.start_timer(5)
            self.start_docking = False
            self.docking_timer = False
            self.second_timer = True
        
        elif self.start_docking:
            if self.docking_hdg - 5 > self.communication.heading:
                rospy.loginfo("Correcting heading starboard...")
                self.communication.rotate_x_deg(self.docking_hdg, 10)
            elif self.docking_hdg + 5 < self.communication.heading:
                rospy.loginfo("Correcting heading port...")
                self.communication.rotate_x_deg(self.docking_hdg, -10)
            elif self.timer_reached() and self.first_timer:
                rospy.loginfo("Starting docking timer!")
                self.start_timer(30)
                self.first_timer = False
                self.docking_timer = True
            else:
                rospy.loginfo("Docking...")
                self.communication.change_mode("LOITER")
                #self.communication.move_sideways(0.5)

        elif self.second_timer:
            if self.timer_reached():
                rospy.loginfo("Undocking timer reached, mission complete!")
                self.communication.RTL()
            else:
                rospy.loginfo("Undocking...")
                #self.communication.move_sideways(-0.5)
            
        elif self.communication.waypoint_reached():
            self.start_docking = True
            self.start_timer(5)
            self.first_timer = True
        
        else:
            rospy.loginfo("Waypoints set, waiting to reach waypoint before setting next waypoint.")
            time.sleep(0.1)

    def _image_and_depth_map(self):
        rospy.logdebug("Getting image and depth map...")

        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.img_left, sl.VIEW.LEFT)
            self.np_img_left = self.img_left.get_data()
            self.np_img_left = self.np_img_left[:,:,:3] # Remove alpha channel
            self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
            self.depth_img = self.depth_map.get_data()
        else:
            rospy.logfatal("No image! Exiting program...")
            exit(1)

    def _get_detections(self):
        rospy.logdebug("Getting detections...")
        return self.model.predict(source=self.np_img_left, conf=0.2, show=self.DEBUG_CAM) 
    
    def detection_results(self):
        self._image_and_depth_map()
        results = self._get_detections()
        self.depth_is_nan = False

        rospy.logdebug("Getting detection results...")

        self.color_list = []
        self.depth_list = []
        self.bearing_list = []

        for result in results:
            for box in result.boxes.xyxy:
                center = ((box[2].item() - box[0].item()) / 2 + box[0].item(), 
                          (box[3].item() - box[1].item()) / 2 + box[1].item())
                if math.isnan(self.depth_img[int(center[1])][int(center[0])]):
                    self.depth_is_nan = True
                self.depth_list.append(self.depth_img[int(center[1])][int(center[0])])
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

    def check_gate_orientation(self): # Assumes check_buoy_gate function is already called
        try:
            if self.closest_color == "red_buoy":
                if (self.closest_bearing - self.second_closest_bearing) < 0: 
                    rospy.logdebug("Red buoy is on the left, and green buoy is on the right.")
                    return True
                else:
                    rospy.logdebug("Green buoy is on the left, and red buoy is on the right.") 
                    return False
            else:
                if (self.closest_bearing - self.second_closest_bearing) > 0:                     
                   rospy.logdebug("Red buoy is on the left, and green buoy is on the right.")
                   return True                                                                  
                else:                                                                            
                   rospy.logdebug("Green buoy is on the left, and red buoy is on the right.")
                   return False    
        except TypeError:
            rospy.logerr("Can't check gate orientation!")

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

            if not self.second_is_none:
                second_closest_bearing_rad = np.radians(drone_heading + self.second_closest_bearing) # With respect to North
                second_closest_buoy_lat = np.arcsin(np.sin(drone_lat_rad) * np.cos(self.second_closest_dist/R) + np.cos(drone_lat_rad) * np.sin(self.second_closest_dist/R) * np.cos(second_closest_bearing_rad))
                second_closest_buoy_lon = drone_lon_rad + np.arctan2(np.sin(second_closest_bearing_rad) * np.sin(self.second_closest_dist/R) * np.cos(drone_lat_rad), np.cos(self.second_closest_dist/R) - np.sin(drone_lat_rad) * np.sin(second_closest_buoy_lat))
                self.second_closest_GPS.append(round(np.degrees(second_closest_buoy_lat), self.GPS_round))
                self.second_closest_GPS.append(round(np.degrees(second_closest_buoy_lon), self.GPS_round)) 
                rospy.logdebug("Second closest buoy GPS: " + str(self.second_closest_GPS))
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
    def hdg_rel_to_bearing(_bearing, rel_angle, drone_hdg): # _bearing needs to be in range [-180, 180]
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
        else:
            rospy.logerr("Error in hdg_rel_to_bearing! _bearing: " + str(_bearing) + " drone_hdg: " + str(drone_hdg) + " out_heading: " + str(out_heading) + " rel_angle: " + str(rel_angle))
        return out_heading
    
    def reset_search_vars(self):
        self.stb_clear = False
        self.port_clear = False
        self.search_iter = 0

    def im_lost(self):
        if self.search_iter >= self.search_max_iter:
            return True
        else:
            return False

    def start_timer(self, sec):
        self.start_time = datetime.now() + timedelta(seconds=sec)

    def timer_reached(self):
        if datetime.now() >= self.start_time:
            return True
        else:
            return False

    def gate_wp(self):
        self.wp_lat = round((self.closest_GPS[0] + self.second_closest_GPS[0]) / 2, self.GPS_round)
        self.wp_lon = round((self.closest_GPS[1] + self.second_closest_GPS[1]) / 2, self.GPS_round)

        self.buoys_bearing = self.two_points_bearing(self.closest_GPS[0], self.closest_GPS[1], self.second_closest_GPS[0], self.second_closest_GPS[1])
        rospy.logdebug("Buoys bearing: " + str(self.buoys_bearing))
        out_gate_heading = self.hdg_rel_to_bearing(self.buoys_bearing, 90, self.communication.heading)
        rospy.logdebug("Out of gate heading: " + str(out_gate_heading))
        self.wp_lat_out, self.wp_lon_out = self.dist_to_GPS_cords(self.out_dist, out_gate_heading, self.wp_lat, self.wp_lon)

        self.wp_lat_out_ret = 2*self.wp_lat - self.wp_lat_out
        self.wp_lon_out_ret = 2*self.wp_lon - self.wp_lon_out

        rospy.loginfo("Waypoint set at: " + "(" +  str(self.wp_lat) + ", " + str(self.wp_lon) + ")")
        rospy.loginfo("Continuing " + str(self.out_dist) + "m to get clear of gate, to waypoint: " + "(" +  str(self.wp_lat_out) + ", " + str(self.wp_lon_out) + ")")
    
    def obstacle_channel_yellow_buoy(self):
        drone_buoy_bearing = self.two_points_bearing(self.communication.lat, self.communication.lon, self.closest_GPS[0], self.closest_GPS[1])
        
        dist_to_wp = np.sqrt(self.closest_dist**2 + self.wp_dist_from_buoy**2)
        if self.closest_bearing < 0:
            angle_buoy_wp = -np.degrees(np.arctan2(self.wp_dist_from_buoy, self.closest_dist))
        else:
            angle_buoy_wp = np.degrees(np.arctan2(self.wp_dist_from_buoy, self.closest_dist))
        self.wp_yellow_buoy_lat, self.wp_yellow_buoy_lon = self.dist_to_GPS_cords(dist_to_wp, drone_buoy_bearing + angle_buoy_wp, self.communication.lat, self.communication.lon)

        rospy.loginfo("Navigating around yellow_buoy, waypoint set at: " + "(" +  str(self.wp_yellow_buoy_lat) + ", " + str(self.wp_yellow_buoy_lon) + ")")
    
    def speed_gate_yellow_buoy(self):
        drone_buoy_bearing = self.two_points_bearing(self.communication.lat, self.communication.lon, self.closest_GPS[0], self.closest_GPS[1])

        if self.closest_bearing < 0: # Decide which side of the buoy to go around first
            x = -1
        else:
            x = 1
        
        dist_to_wp_1 = np.sqrt(self.closest_dist**2 + self.wp_dist_from_buoy**2)
        angle_buoy_wp_1 = x*np.degrees(np.arctan2(self.wp_dist_from_buoy, self.closest_dist))
        self.yellow_buoy_lat_1, self.yellow_buoy_lon_1 = self.dist_to_GPS_cords(dist_to_wp_1, drone_buoy_bearing + angle_buoy_wp_1, self.communication.lat, self.communication.lon)

        dist_to_wp_2 = self.closest_dist + self.wp_dist_from_buoy
        self.yellow_buoy_lat_2, self.yellow_buoy_lon_2 = self.dist_to_GPS_cords(dist_to_wp_2, drone_buoy_bearing, self.communication.lat, self.communication.lon)

        dist_to_wp_3 = np.sqrt(self.closest_dist**2 + (-self.wp_dist_from_buoy**2))
        angle_buoy_wp_3 = -x*np.degrees(np.arctan2(self.wp_dist_from_buoy, self.closest_dist))
        self.yellow_buoy_lat_3, self.yellow_buoy_lon_3 = self.dist_to_GPS_cords(dist_to_wp_3, drone_buoy_bearing + angle_buoy_wp_3, self.communication.lat, self.communication.lon)

# ---------------------------------------------- Autopilot Communication ---------------------------------------------- #

class apCommunication:

    max_speed = 1.6 # Vessel max speed in m/s

    def __init__(self): 

        self.pub_vel = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.cmd_vel = Twist()
        self.pub_guided_wp = rospy.Publisher("/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)
        self.guided_wp = GlobalPositionTarget()

        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.sub_GPS = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.sub_heading = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.heading_callback)
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

        self.mode = self.ini_mode

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

    def wps_callback(self, msg):
        self.curr_seq = msg.current_seq
        self.wps = msg.waypoints
        rospy.logdebug("Waypoint list: " + str(self.wps))

    def wp_reached_callback(self, msg):
        self.reached_seq = msg.wp_seq
        self.wp_reached = True 
        rospy.loginfo("Waypoint reached!")
        rospy.logdebug("Waypoint reached: " + str(self.reached_seq))

    def auto_wp_reached(self): 
        if self.mode == "AUTO" and (self.curr_seq == self.reached_seq):
            time.sleep(0.1)
            if self.curr_seq == self.reached_seq:
                rospy.loginfo("AUTO waypoint reached!")
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
        wp.command = cmd  # 16 = Nav command, 20=RTL
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
    
    def clear_waypoints(self):
        rospy.wait_for_service('mavros/mission/clear')
        try:
            response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            self.wp_list = []
            self.make_waypoint(0, 0) # Add dummy waypoint to list, ignored by FCU	
            return response.call().success
        except rospy.ServiceException as e:
            rospy.logerr("Clear waypoint failed: %s"%e)
            return False        

    def send_guided_wp(self, lat, lon):
        self.guided_wp.latitude = lat
        self.guided_wp.longitude = lon
        while not self.ctrl_c:
            connections = self.pub_guided_wp.get_num_connections()
            if connections > 0:
                self.pub_guided_wp.publish(self.guided_wp)
                rospy.loginfo("GUIDED waypoint published!")
                self.wp_set = True
                break
            else:
                rospy.logdebug("No subscribers to guided_wp yet, waiting to try again!")

    def clear_guided_wp(self):
        self.change_mode("MANUAL")
        self.change_mode("GUIDED")

    def RTL(self):
        self.change_mode("GUIDED")
        self.clear_waypoints()
        self.make_waypoint(0, 0, cmd=20)
        self.send_waypoint()
        self.change_mode("AUTO")

    def rotate_x_deg(self, x_deg, rate): # Positive x_deg is starboard, negative is port
        tol = round(1/4 * np.abs(rate), 2) # Tolerance is 1/4 of the rate
        targ_hdg_low = self.heading + x_deg - tol
        targ_hdg_high = self.heading + x_deg + tol

        if targ_hdg_low >= 360:
            targ_hdg_low -= 360
        elif targ_hdg_low < 0:
            targ_hdg_low += 360

        if targ_hdg_high >= 360:
            targ_hdg_high -= 360
        elif targ_hdg_high < 0:
            targ_hdg_high += 360

        while not self.ctrl_c: 
            if (self.heading > targ_hdg_low) and (self.heading < targ_hdg_high):
                break
            else:
                self.rotate(rate)
                rospy.sleep(0.1)
            rospy.logdebug("Current heading: " + str(self.heading) + ", Target heading: " + str(targ_hdg_high-tol))
        self.stop()
        
    def rotate(self, deg_s): # Positive deg_s is starboard, negative is port
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = np.radians(deg_s)
        self.pub_vel.publish(self.cmd_vel)
    
    def move_forward(self, lin_vel):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = lin_vel
        self.cmd_vel.angular.z = 0
        self.pub_vel.publish(self.cmd_vel)
   
    def move_sideways(self, lin_vel):
        self.cmd_vel.linear.x = lin_vel
        self.cmd_vel.linear.y = 0
        self.cmd_vel.angular.z = 0#np.radians(60)
        self.pub_vel.publish(self.cmd_vel)

    def stop(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        while not self.ctrl_c:
            connections = self.pub_vel.get_num_connections()
            if connections > 0:
                self.pub_vel.publish(self.cmd_vel)
                rospy.loginfo("Stopping motors")
                break
            else:
                rospy.logdebug("No subscribers to pub_vel yet, trying again...")
