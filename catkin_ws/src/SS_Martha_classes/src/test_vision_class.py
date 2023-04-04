from martha_classes import droneLidar

MarthaVision = droneVision()

img_left, depth_map = MarthaVision.get_image_and_depth_map()

detections = MarthaVision.get_detections()
print(detections)

# buoy_1_distance = detections[0][2][0]
# buoy_2_distance = detections[0][2][1]
# buoy_1_bearing = MarthaVision.get_det_bearing(detections[0][0][0])
# buoy_2_bearing = MarthaVision.get_det_bearing(detections[0][0][1])
# buoy_1_lat, buoy_1_lon = MarthaVision.det_GPS_loc(buoy_distance, buoy_bearing, Martha_lat, Martha_lon)
# buoy_2_lat, buoy_2_lon = MarthaVision.det_GPS_loc(buoy_2_distance, buoy_2_bearing, Martha_lat, Martha_lon)
# wp = MarthaVision.set_waypoint(buoy_1_lat, buoy_1_lon, buoy_2_lat, buoy_2_lon)

MarthaLidar = droneLidar()
