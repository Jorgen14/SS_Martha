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
    print("Buoys: ", MarthaVision.buoy_color)
    print("Buoy depth: ", MarthaVision.buoy_depth)
    print("Buoy bearing: ", MarthaVision.buoy_bearing)

    MarthaVision.get_closest_buoy()
    print("Closest buoy color: ", MarthaVision.closest_color)
    print("Closest buoy distance: ", MarthaVision.closest_dist)
    print("Closest buoy bearing: ", MarthaVision.closest_bearing)

    MarthaVision.get_2nd_closest_buoy()
    print("2nd closest buoy color: ", MarthaVision.second_closest_color)
    print("2nd closest buoy distance: ", MarthaVision.second_closest_dist)
    print("2nd closest buoy bearing: ", MarthaVision.second_closest_bearing)

    print("Buoy gate detected: ", MarthaVision.check_buoy_gate())

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
