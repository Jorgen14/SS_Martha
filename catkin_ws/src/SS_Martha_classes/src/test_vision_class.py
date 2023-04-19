import cv2 as cv
from martha_classes import droneVision
from keyboard import is_pressed


MarthaVision = droneVision()

MarthaVision.DEBUG = True

while True:
    # lat 
    # lon
    # heading
    # img_left, depth_map = MarthaVision.get_image_and_depth_map()

    # detections = MarthaVision.get_detections(img_left)

    MarthaVision.get_det_results()
    print("Buoys: ", MarthaVision.det_name)
    print("Buoy depth: ", MarthaVision.det_depth)
    print("Buoy bearing: ", MarthaVision.det_bearing)

    #detection_depth = MarthaVision.get_det_depth(detection_center)
    #gps_loc = MarthaVision.det_GPS_loc(detection_center, lat, lon, heading)

    if is_pressed('q'):
        break

MarthaVision.zed.close()

# buoy_1_distance = detections[0][2][0]
# buoy_2_distance = detections[0][2][1]
# buoy_1_bearing = MarthaVision.get_det_bearing(detections[0][0][0])
# buoy_2_bearing = MarthaVision.get_det_bearing(detections[0][0][1])
# buoy_1_lat, buoy_1_lon = MarthaVision.det_GPS_loc(buoy_distance, buoy_bearing, Martha_lat, Martha_lon)
# buoy_2_lat, buoy_2_lon = MarthaVision.det_GPS_loc(buoy_2_distance, buoy_2_bearing, Martha_lat, Martha_lon)
# wp = MarthaVision.set_waypoint(buoy_1_lat, buoy_1_lon, buoy_2_lat, buoy_2_lon)
