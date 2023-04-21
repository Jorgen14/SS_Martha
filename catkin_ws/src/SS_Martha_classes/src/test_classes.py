import cv2 as cv
from martha_classes import droneVision, droneData
from keyboard import is_pressed

TEST_VISION = False
TEST_DATA = True
MarthaVision = droneVision(DEBUG=True)
MarthaData = droneData()

while True:
    if TEST_VISION:
        MarthaVision.get_det_results()
        #print("Buoys: ", MarthaVision.buoy_color)
        #print("Buoy depth: ", MarthaVision.buoy_depth)
        #print("Buoy bearing: ", MarthaVision.buoy_bearing)

        """
        MarthaVision.get_closest_buoy()
        print("Closest buoy color: ", MarthaVision.closest_color)
        print("Closest buoy distance: ", MarthaVision.closest_dist)
        print("Closest buoy bearing: ", MarthaVision.closest_bearing)

        MarthaVision.get_2nd_closest_buoy()
        print("2nd closest buoy color: ", MarthaVision.second_closest_color)
        print("2nd closest buoy distance: ", MarthaVision.second_closest_dist)
        print("2nd closest buoy bearing: ", MarthaVision.second_closest_bearing)
        """

        print("Buoy gate detected: ", MarthaVision.check_buoy_gate())
        
        # MarthaVision.buoy_GPS_loc(MarthaData.drone_lat, MarthaData.drone_lon, MarthaData.drone_heading)
        # print("Buoy GPS location: ", MarthaVision.closest_GPS, MarthaVision.second_closest_GPS)

    if TEST_DATA:
        print("Latitude, longitude, heading: ", MarthaData.drone_lat, MarthaData.drone_lon, MarthaData.drone_heading)
        print("")
        print("Linear speed x and y: ", MarthaData.lin_vel_x, MarthaData.lin_vel_y)
        print("")
        print("Angular speed z: ", MarthaData.ang_vel_z)
        print("")
        print("Waypoint reached: ", MarthaData.waypoint_reached)
        print("")

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
