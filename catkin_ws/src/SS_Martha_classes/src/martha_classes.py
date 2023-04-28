import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import Waypoint, WaypointReached, WaypointList
from mavros_msgs.srv import WaypointPush, WaypointClear

class droneVision:

    GPS_round = 6
    rel_dist_thresh = 3.0 # Maximum relative distance between buoys

    def __init__(self, DEBUG=False, DEBUG_CAM=False):
        self.DEBUG = DEBUG
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
        self.height = self.zed.get_camera_information().camera_resolution.height
        self.hfov = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.h_fov
        self.cx = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.cx
        self.baseline = self.zed.get_camera_information().camera_configuration.calibration_parameters.get_camera_baseline
        self.lamda_x = self.hfov / self.width

        self.img_left = sl.Mat()
        self.depth_map = sl.Mat()

        self.model = YOLO('/home/navo/GitHub/SS_Martha/YOLOv8/buoy_s.pt') 

        self.communication = apCommunication()

        rospy.loginfo("ZED Camera Initialized!")

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
        return self.model.predict(source=self.np_img_left, conf=0.5, show=self.DEBUG_CAM) 
    
    def detection_results(self):
        self._image_and_depth_map()
        results = self._get_detections()

        rospy.logdebug("Getting detection results...")

        self.color_list = []
        self.depth_list = []
        self.bearing_list = []

        for result in results:
            for box in result.boxes.xyxy:
                center = ((box[2].item() - box[0].item()) / 2 + box[0].item(), 
                          (box[3].item() - box[1].item()) / 2 + box[1].item())
                self.depth_list.append(self.depth_img[int(center[1])][int(center[0])])
                Tx = int(center[0]) - self.cx
                theta = Tx * self.lamda_x
                self.bearing_list.append(theta)
            for d_cls in result.boxes.cls:
                self.color_list.append(self.model.names[int(d_cls)])

            rospy.logdebug("Results prosessed!")
    
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
            self.closeset_dist = round(self.closest_dist, 2)
            self.closest_color = self.color_list[closest_index]
            self.closest_bearing = round(self.bearing_list[closest_index], 2)
            self.closest_is_none = False
            rospy.loginfo("Distance to " + self.closest_color + ": " + str(self.closest_dist) + "m " + "at bearing: " + str(self.closest_bearing) + " degrees.")
        except IndexError:
            rospy.logwarn("No buoys detected!")
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
            rospy.loginfo("Distance to " + self.second_closest_color + ": " + str(self.second_closest_dist) + "m " + "at bearing: " + str(self.second_closest_bearing) + " degrees.")
        except IndexError:
            if self.closest_color == "yellow_buoy":
                rospy.loginfo("Only yellow buoy detected.")
            else: 
                rospy.logwarn("Only one buoy detected!")
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

    def check_gate_orientation(self):
        if (self.closest_bearing - self.second_closest_bearing) < 0:
            rospy.logdebug("Green buoy is closest and red buoy is second closest.")
            rospy.logdebug("Green buoy is on the left, and red buoy is on the right.")
            return True
        else:
            rospy.logdebug("Red buoy is closest and green buoy is second closest.")
            rospy.logdebug("Red buoy is on the left, and green buoy is on the right.")
            return False

    def check_rel_dist(self):
        a = self.closest_dist
        c = self.second_closest_dist
        beta = np.radians(self.closest_bearing - self.second_closest_bearing)
        rel_dist = np.sqrt(a**2 + c**2 - 2*a*c*np.cos(beta))

        if rel_dist < self.rel_dist_thresh:
            rospy.logdebug("Relative distance is less than threshold.")
            return True
        else:
            rospy.logdebug("Relative distance between buoys is greater than threshold.")
            return False
        
    def buoy_GPS_loc(self, R=6371e3):
        self.closest_GPS = []
        self.second_closest_GPS = []

        drone_lat_rad = np.radians(self.communication.lat)
        drone_lon_rad = np.radians(self.communication.lon)
        drone_heading = self.communication.heading

        if not self.closest_is_none:
            closest_bearing_rad = np.radians(drone_heading + self.closest_bearing) # With respect to North
            closest_buoy_lat = round(np.arcsin(np.sin(drone_lat_rad) * np.cos(self.closest_dist/R) + np.cos(drone_lat_rad) * np.sin(self.closest_dist/R) * np.cos(closest_bearing_rad)), self.GPS_round)
            closest_buoy_lon = round(drone_lon_rad + np.arctan2(np.sin(closest_bearing_rad) * np.sin(self.closest_dist/R) * np.cos(drone_lat_rad), np.cos(self.closest_dist/R) - np.sin(drone_lat_rad) * np.sin(closest_buoy_lat)), self.GPS_round)
            self.closest_GPS.append(np.degrees(closest_buoy_lat))
            self.closest_GPS.append(np.degrees(closest_buoy_lon)) 
            rospy.logdebug("Closest buoy GPS: " + str(self.closest_GPS))

            if not self.second_is_none and not self.closest_color == 'yellow_buoy':
                second_closest_bearing_rad = np.radians(drone_heading + self.second_closest_bearing) # With respect to North
                second_closest_buoy_lat = round(np.arcsin(np.sin(drone_lat_rad) * np.cos(self.second_closest_dist/R) + np.cos(drone_lat_rad) * np.sin(self.second_closest_dist/R) * np.cos(second_closest_bearing_rad)), self.GPS_round)
                second_closest_buoy_lon = round(drone_lon_rad + np.arctan2(np.sin(second_closest_bearing_rad) * np.sin(self.second_closest_dist/R) * np.cos(drone_lat_rad), np.cos(self.second_closest_dist/R) - np.sin(drone_lat_rad) * np.sin(second_closest_buoy_lat)), self.GPS_round)
                self.second_closest_GPS.append(np.degrees(second_closest_buoy_lat))
                self.second_closest_GPS.append(np.degrees(second_closest_buoy_lon)) 
                rospy.logdebug("Second closest buoy GPS: " + str(self.second_closest_GPS))
            elif self.closest_color == 'yellow_buoy':
                rospy.loginfo("Yellow buoy detected.")
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

        return round(np.degrees(lat2), 6), round(np.degrees(lon2), 6)

    # def buoy_GPS_loc(self, GPS_list, R=6371e3):
    #     drone_lat_rad = np.radians(self.communication.lat)
    #     drone_lon_rad = np.radians(self.communication.lon)
    #     drone_heading = self.communication.heading

    #     closest_bearing_rad = np.radians(drone_heading + self.closest_bearing) # With respect to North
    #     closest_buoy_lat = round(np.arcsin(np.sin(drone_lat_rad) * np.cos(self.closest_dist/R) + np.cos(drone_lat_rad) * np.sin(self.closest_dist/R) * np.cos(closest_bearing_rad)), self.GPS_round)
    #     closest_buoy_lon = round(drone_lon_rad + np.arctan2(np.sin(closest_bearing_rad) * np.sin(self.closest_dist/R) * np.cos(drone_lat_rad), np.cos(self.closest_dist/R) - np.sin(drone_lat_rad) * np.sin(closest_buoy_lat)), self.GPS_round)
    #     self.closest_GPS.append(np.degrees(closest_buoy_lat))
    #     self.closest_GPS.append(np.degrees(closest_buoy_lon))

    def obstacle_channel_gate(self):
        self.wp_lat = round((self.closest_GPS[0] + self.second_closest_GPS[0]) / 2, self.GPS_round)
        self.wp_lon = round((self.closest_GPS[1] + self.second_closest_GPS[1]) / 2, self.GPS_round)

        rospy.loginfo("Waypoint set at: " + "(" +  str(self.wp_lat) + ", " + str(self.wp_lon) + ")")

        closest_rad_lat = np.radians(self.closest_GPS[0])
        closest_rad_lon = np.radians(self.closest_GPS[1])
        second_rad_lat = np.radians(self.second_closest_GPS[0])
        second_rad_lon = np.radians(self.second_closest_GPS[1])
        
        x = np.cos(second_rad_lat) * np.sin(second_rad_lon - closest_rad_lon)
        y = np.cos(closest_rad_lat) * np.sin(second_rad_lat) - np.sin(closest_rad_lat) * np.cos(second_rad_lat) * np.cos(second_rad_lon - closest_rad_lon)
        buoys_bearing = np.arctan2(x, y)

        if buoys_bearing < 0:
            buoys_bearing += 360

        if self.communication.heading > 180:
            out_heading = np.degrees(buoys_bearing) + 90
            if out_heading > 360:
                out_heading -= 360
        else:
            out_heading = np.degrees(buoys_bearing) - 90
            if out_heading < 0:
                out_heading += 360

        # if buoys_bearing > 0 and buoys_bearing < 90:
        #     if self.communication.heading > 180:
        #         out_heading = np.degrees(buoys_bearing) + 90
        #     else:
        #         out_heading = np.degrees(buoys_bearing) - 90
        # elif buoys_bearing > 90 and buoys_bearing < 180:
        #     if self.communication.heading > 180:
        #         out_heading = np.degrees(buoys_bearing) - 90
        #     else:
        #         out_heading = np.degrees(buoys_bearing) + 90
        # elif buoys_bearing < 0 and buoys_bearing > -90:
        #     if self.communication.heading > 180:
        #         out_heading = np.degrees(buoys_bearing) + 90
        #     else:
        #         out_heading = np.degrees(buoys_bearing) - 90

        out_dist = 1.0 # meters

        self.wp_lat_out, self.wp_lon_out = self.dist_to_GPS_cords(out_dist, out_heading, self.wp_lat, self.wp_lon)
    
    # def obstacle_channel_yellow(self, position, bearing):

    
    def nav_channel(self):
        self.detection_results()
        self.get_closest_buoy()
        self.get_2nd_closest_buoy()

        if self.check_buoy_gate() and self.check_gate_orientation():
            rospy.loginfo("Gate detected.")
            self.buoy_GPS_loc()
            self.obstacle_channel_gate()
            self.communication.send_waypoint()

        elif self.closest_color == "yellow_buoy" and self.second_is_none:
                rospy.loginfo("Yellow buoy detected, navigating around.")

        elif self.closest_color == "yellow_buoy" and not self.second_is_none:
            rospy.loginfo("Yellow buoy detected.")
            if self.check_rel_dist():
                rospy.loginfo("Yellow buoy detected, possibly false negative.")
                rospy.loginfo("Moving closer to check again.")
            else:
                rospy.loginfo("Yellow buoy detected, navigating around.")

        elif self.closest_color == "red_buoy" and self.second_is_none:
            rospy.loginfo("Only detected a red buoy, navigating closer.")

        elif self.closest_color == "green_buoy" and not self.second_is_none:
            rospy.loginfo("Only detected a green buoy, navigating closer.")

        elif not self.check_gate_orientation():
            rospy.loginfo("Buoy gate detected, but not in the right orientation.")
            rospy.loginfo("Rotaing 180 degrees.")

        else:
            rospy.loginfo("No detections, moving forward to check again.")



class apCommunication: # Communication with the autopilot
    def __init__(self, DEBUG=False):
        self.DEBUG = DEBUG

        self.vision = droneVision() 

        self.sub_GPS = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.sub_heading = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.heading_callback)
        self.sub_vel = rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.vel_callback)
        self.sub_wp_reached = rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.wp_reached_callback)
        # self.sub_wps = rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self.wps_callback)
        
        self.wl = []
        self.lat = None
        self.lon = None
        self.heading = None
        self.lin_vel_x = None
        self.lin_vel_y = None
        self.ang_vel_z = None
        self.wp_reached = None
        self.wp_list = None
        self.wp_set = False
        self.check = None

        self.rate = rospy.Rate(10)

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

    def wp_reached_callback(self, msg):
        self.wp_reached = msg.wp_seq

        rospy.logdebug("Waypoint reached: " + str(self.wp_reached))

    # def wps_callback(self, msg):
    #     self.wp_list = msg.waypoints

    #     rospy.logdebug("Waypoint list: " + str(self.wp_list))

    def send_waypoint(self):
        wp = Waypoint()
        wp.frame = 0 # Global frame
        wp.command = 16  # Takeoff command
        wp.is_current = True
        wp.autocontinue = True
        wp.param1 = 0  # HOLD time at WP
        wp.param2 = 0  # Acceptance radius, if inside WP count as reached
        wp.param3 = 0  # Pass Radius. If 0 go through WP
        wp.param4 = 0 # float('nan')  # Yaw, 0 for our USV situation
        wp.x_lat = self.vision.wp_lat # Latitude
        wp.y_long = self.vision.wp_lon # Longitude
        wp.z_alt = 0
        self.wl.append(wp)

        rospy.wait_for_service('mavros/mission/push')
        try:
            serviceReq = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            serviceRes = serviceReq(start_index=0, waypoints=self.wl)
            flag = serviceRes.success
            if flag:
                self.wp_set = True
                print("Waypoint successfully pushed")
            else:
                print('FAILURE: PUSHING WP \n')		
        except rospy.ServiceException as e:
            rospy.loginfo("Failed to send WayPoint failed: %s\n" %e)

    def clear_waypoints(self):
        rospy.wait_for_service('mavros/mission/clear')
        try:
            response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)		
            return response.call().success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
