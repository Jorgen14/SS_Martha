import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import WaypointReached

class droneVision:
    def __init__(self, DEBUG=False, DEBUG_CAM=False):
        self.DEBUG = DEBUG
        self.DEBUG_CAM = DEBUG_CAM

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.sdk_verbose = True

        self.runtime_params = sl.RuntimeParameters()
        self.zed_status = self.zed.open(init_params)

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
        self.img_depth = sl.Mat()

        self.model = YOLO('buoy_detect.pt') 

        print("ZED Camera initialized")

    def get_image_and_depth_map(self): # Return anything?
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:

            self.zed.retrieve_image(self.img_left, sl.VIEW.LEFT)
            np_img_left_4ch = self.img_left.get_data()
            self.np_img_left = np_img_left_4ch[:,:,:3] # Remove alpha channel

            self.zed.retrieve_measure(self.img_depth, sl.MEASURE.DEPTH) # May need to change resolution
            self.depth_map = self.img_depth.get_data()

            return self.np_img_left, self.depth_map
        else:
            print("Error grabbing image")
            exit(1)

    def get_detections(self, image):
        return self.model.predict(source=image, conf=0.5, show=self.DEBUG_CAM) 
    
    def get_det_results(self):
        if self.DEBUG:
            print("Getting results from detections...")
            print("")

        self.get_image_and_depth_map()
        results = self.get_detections(self.np_img_left)

        self.buoy_color = []
        self.buoy_depth = []
        self.buoy_bearing = []

        for result in results:
            for box in result.boxes.xyxy:
                center = ((box[2].item() - box[0].item()) / 2 + box[0].item(), 
                          (box[3].item() - box[1].item()) / 2 + box[1].item())
                self.buoy_depth.append(self.depth_map[int(center[1])][int(center[0])])
                Tx = int(center[0]) - self.cx
                theta = Tx * self.lamda_x
                self.buoy_bearing.append(theta)
            for d_cls in result.boxes.cls:
                self.buoy_color.append(self.model.names[int(d_cls)])

        return self.buoy_color, self.buoy_depth, self.buoy_bearing
    
    def get_closest_buoy(self):
        if self.DEBUG:
            print("Getting closest buoy...")
            print("")

        self.closest_color = None
        self.closest_dist = None
        self.closest_bearing = None

        depth_list_sorted = sorted(self.buoy_depth)

        try:
            self.closest_dist = depth_list_sorted[0]
            closest_index = self.buoy_depth.index(self.closest_dist)
            self.closest_color = self.buoy_color[closest_index]
            self.closest_bearing = self.buoy_bearing[closest_index]
        except IndexError:
            print("Not enough buoys detected!")
            pass

        return self.closest_color, self.closest_dist, self.closest_bearing
    
    def get_2nd_closest_buoy(self):
        if self.DEBUG:
            print("Getting second closest buoy")
            print("")

        self.second_closest_color = None
        self.second_closest_dist = None
        self.second_closest_bearing = None

        depth_list_sorted = sorted(self.buoy_depth)

        try:
            self.second_closest_dist = depth_list_sorted[1]
            second_closest_index = self.buoy_depth.index(self.second_closest_dist)
            self.second_closest_color = self.buoy_color[second_closest_index]
            self.second_closest_bearing = self.buoy_bearing[second_closest_index]
        except IndexError:
            print("Not enough buoys detected")
            pass

        return self.second_closest_color, self.second_closest_dist, self.second_closest_bearing
    
    def check_buoy_gate(self):
        self.get_closest_buoy()
        self.get_2nd_closest_buoy()

        if self.DEBUG:
            print("Buoys detected: ", self.closest_color, "and ", self.second_closest_color)
            print("")

        if self.closest_color == 'green_buoy' and self.second_closest_color == 'red_buoy':
            return True
        elif self.closest_color == 'red_buoy' and self.second_closest_color == 'green_buoy':
            return True
        else:
            return False # If false look for yellow buoy, -> get yellow buoy GPS loc from own function
        
    def buoy_GPS_loc(self, drone_lat, drone_lon, drone_heading, R=6371e3):
        self.get_closest_buoy()
        self.get_2nd_closest_buoy()
        
        self.closest_GPS = []
        self.second_closest_GPS = []

        drone_lat_rad = np.radians(drone_lat)
        drone_lon_rad = np.radians(drone_lon)
        closest_bearing_rad = np.radians(drone_heading + self.closest_bearing) # With respect to North
        second_closest_bearing_rad = np.radians(drone_heading + self.second_closest_bearing)

        closest_buoy_lat = np.arcsin(np.sin(drone_lat_rad) * np.cos(self.closest_dist/R) + np.cos(drone_lat_rad) * np.sin(self.closest_dist/R) * np.cos(closest_bearing_rad))
        closest_buoy_lon = drone_lon_rad + np.arctan2(np.sin(closest_bearing_rad) * np.sin(self.closest_dist/R) * np.cos(drone_lat_rad), np.cos(self.closest_dist/R) - np.sin(drone_lat_rad) * np.sin(closest_buoy_lat))
        self.closest_GPS.append(np.degrees(closest_buoy_lat), np.degrees(closest_buoy_lon))

        second_closest_buoy_lat = np.arcsin(np.sin(drone_lat_rad) * np.cos(self.second_closest_dist/R) + np.cos(drone_lat_rad) * np.sin(self.second_closest_dist/R) * np.cos(second_closest_bearing_rad))
        second_closest_buoy_lon = drone_lon_rad + np.arctan2(np.sin(second_closest_bearing_rad) * np.sin(self.second_closest_dist/R) * np.cos(drone_lat_rad), np.cos(self.second_closest_dist/R) - np.sin(drone_lat_rad) * np.sin(second_closest_buoy_lat))
        self.second_closest_GPS.append(np.degrees(second_closest_buoy_lat), np.degrees(second_closest_buoy_lon))

        return self.closest_GPS, self.second_closest_GPS
    
    # def yellow_buoy_GPS_loc(self): # Make transit to waypoint function, and look for yellow buoy while transiting

    def set_waypoint(self):
        if self.check_buoy_gate():
            self.buoy_GPS_loc()
            self.wp_lat = (self.closest_GPS[0] + self.second_closest_GPS[0]) / 2
            self.wp_lon = (self.closest_GPS[1] + self.second_closest_GPS[1]) / 2
            return self.wp_lat, self.wp_lon
        # elif look for yellow buoy
        else:  
            print("No buoy gate found. Searching for gate...")



class droneData:
    def __init__(self, DEBUG=False):
        self.DEBUG = DEBUG

        self.sub_GPS = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.sub_heading = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.heading_callback)
        self.sub_vel = rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.vel_callback)
        self.sub_wp_reached = rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.wp_reached_callback)
        
        self.drone_lat = None
        self.drone_lon = None
        self.drone_heading = None
        self.lin_vel_x = None
        self.lin_vel_y = None
        self.ang_vel_z = None
        self.wp_reached = None

    def gps_callback(self, msg):
        self.drone_lat = msg.latitude
        self.drone_lon = msg.longitude
    
    def heading_callback(self, msg):
        self.drone_heading = msg.data
    
    def vel_callback(self, msg):
        self.lin_vel_x = msg.twist.linear.x
        self.lin_vel_y = msg.twist.linear.y
        self.ang_vel_z = msg.twist.angular.z

    def wp_reached_callback(self, msg):
        self.wp_reached = msg.wp_seq
        
