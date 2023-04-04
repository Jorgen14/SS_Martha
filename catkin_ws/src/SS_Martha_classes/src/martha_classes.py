import rospy
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO
from sensor_msgs.msg import LaserScan

class droneVision:
    def __init__(self):
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

        self.width = self.zed.get_camera_information().camera_configuration.resolution.width
        self.height = self.zed.get_camera_information().camera_configuration.resolution.height
        self.hfov = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.h_fov
        self.cx = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.cx
        self.baseline = self.zed.get_camera_information().get_camera_baseline

        self.img_left = sl.Mat()
        self.img_depth = sl.Mat()

        self.model = YOLO('YOLOv8/buoy_detect.pt') 

    def get_image_and_depth_map(self): # Return anything?
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.img_left, sl.VIEW.LEFT)
            self.np_img_left = self.img_left.get_data()
            self.depth_map = self.zed.retrieve_measure(self.img_depth, sl.MEASURE.DEPTH) # May need to change resolution
            return self.np_img_left, self.depth_map
        else:
            print("Error grabbing image")
            exit(1)

    def get_detections(self, image):
        self.detections = self.model.predict(source=image)
        return self.detections

    def get_det_depth(self, x_detection, y_detection):
        return self.depth_map.get_value(x_detection, y_detection)
    
    def get_det_bearing(self, x_detection):
        lamda_x = self.hfov / self.width
        Tx = x_detection - self.cx
        #Tx = x_detection - (self.cx + self.baseline/2) # set Tx from center of stereo pair
        return Tx * lamda_x

    def det_GPS_loc(self, distance, det_bearing, Martha_lat, Martha_lon, R=6371e3):
        lat_rad = np.radians(Martha_lat)
        lon_rad = np.radians(Martha_lon)
        bearing_rad = np.radians(det_bearing)

        det_lat = np.arcsin(np.sin(lat_rad) * np.cos(distance/R) + np.cos(lat_rad) * np.sin(distance/R) * np.cos(bearing_rad))
        det_lon = lon_rad + np.arctan2(np.sin(bearing_rad) * np.sin(distance/R) * np.cos(lat_rad), np.cos(distance/R) - np.sin(lat_rad) * np.sin(det_lat))

        return round(np.degrees(det_lat), 5), round(np.degrees(det_lon), 5)
    
    def set_waypoint(self, det_1_lat, det_1_lon, det_2_lat, det_2_lon):
        wp_lat = (det_1_lat + det_2_lat) / 2
        wp_lon = (det_1_lon + det_2_lon) / 2
        return wp_lat, wp_lon
    

    
class droneLidar:
    def __init__(self):
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.prev_tracking_indexes = []
        self.collision_course_duration = 0
        self.slow_down_counter = 10
        self.emg_stop_counter = 20

    def laser_callback(self, data):
        self.ranges = data.ranges
        self.ang_inc = data.angle_increment

    def close_obj_indexes(self):
        prev_index = None
        j = 0
        self.obj_indexes = []
        for i in range(len(self.ranges)):
            if self.ranges[i] < 20:
                if prev_index is None:
                    self.obj_indexes.append([i])
                    prev_index = i
                else:
                    if i - prev_index <= 1 and self.ranges[i] - self.ranges[prev_index] <= 0.5:
                        self.obj_indexes[j].append(i)
                    else:
                        j += 1
                        self.obj_indexes.append([i])
                    prev_index = i

    def tracking_indexes(self):
        self.tracking_indx = []
        for i in range(len(self.obj_indexes)):
            self.tracking_indx(self.obj_indexes[i][int(len(self.obj_indexes[i])/2)])

    def collision_course(self):
        collision_list = []
        if self.prev_tracking_indexes == []:
            self.prev_tracking_indexes = self.tracking_indx
        else:
            try:
                i = 0
                while self.tracking_indx[i] or self.prev_tracking_indexes[i]:
                    if abs(self.tracking_indx[i] - self.prev_tracking_indexes[i]) <= 5:
                        collision_list.append(1)
                    else:
                        collision_list.append(0)
                    i += 1
            except IndexError:
                pass

        if 1 in collision_list:
            self.collision_course_duration += 1
        else:
            self.collision_course_duration = 0

    def avoid_collision(self):
        if self.collision_course_duration >= self.emg_stop_counter:
            self.emg_stop = True
        elif self.collision_course_duration >= self.slow_down_counter:
            self.slow_down = True
        else:
            self.emg_stop = False
            self.slow_down = False
