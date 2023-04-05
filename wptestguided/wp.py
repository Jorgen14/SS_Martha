import rospy
import time
import math
from math import pi,cos,sin,tan,atan
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Float64
from mavros_msgs.msg import Waypoint    # define waypoints
from mavros_msgs.srv import WaypointPush # push Waypoint

#Wp dictonaries x and y values for 10m setting
#Should be redone for more precise waypoint setting
arming = None ##Might remove this, not decided yet
keys = []
#X values with 10m radius from vessel
y_values = [0,-1.74,-3.42,-5,-6.43,-7.66,-8.66,-9.4,-9.85,-10,-9.85,-9.4,-8.66,-7.66,-6.43,-5,-3.42,
-1.74,0,1.74,3.42,5,6.43,7.66,8.66,9.4,9.85,10,9.85,9.4,8.66,7.66,6.43,5,3.42,1.74,0]
#Y values with 10m radius from vessel
x_values = [10,9.88,9.4,8.66,7.66,6.43,5,3.42,1.74,0,-1.74,-3.42,-5,-6.43,-7.66,-8.66,-9.4,-9.85,-10,-9.85,-9.4,-8.66,-7.66,-6.43,-5,-3.42,-1.74,0,1.74,3.42,5,6.43,7.66,8.66,9.4,9.85,10]
#Key values representing orientation of the vessel
#When orientation matches keys, set the x and y values from the corresponding key as WP
degrees_values = [360,350,340,330,320,310,300,290,280,270,260,250,240,230,220,210,200,190,180,170,160,150,140,130,120,110,100,90,80,70,60,50,40,30,20,10,0]
#store values in dict

a = 0
dict_x = {}
dict_y = {}
for i in degrees_values:
	key = degrees_values[a]
	x = x_values[a]
	y = y_values[a]
	dict_x[key]=x
	dict_y[key]=y
	a += 1	

WP_lon = open(r"WP_lon_.txt","w")
WP_lat = open(r"WP_lat_.txt","w")
WP_x = open(r"WP_xValue.txt","w")
WP_y = open(r"WP_yValue.txt","w")
WP_launch_lon = open(r"WP_launch_lon","w")
WP_launch_lat = open(r"WP_launch_lat","w")
#Global variables
check = None
glob = ''
rc_input = None
current_mode = None
wp_set = False
lat = None
lon = None
#Pre determined modes
MODE_MANUAL = "MANUAL"
MODE_GUIDED = "GUIDED"
MODE_AUTO = "AUTO"
new_lon = None
new_lat = None
#Get values from matching key
def dict_to_float(key, dicts):
	#print dicts.keys()
	if key in dicts.keys():
		res = dicts[key]
		return res

def change_mode(mode):
	global current_mode	
	rospy.wait_for_service('/mavros/set_mode')
	try:
		if current_mode != mode:
			change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			response = change_mode(custom_mode=mode)
			rospy.loginfo(response)
			#mode_change = rospy.ServiceProxy('/mavros/set_mode',SetMode)
			#mode_change(0,mode)
			#current_mode = mode
	except rospy.ServiceException as e:
		print("Mode change failed: %s" %e)

#Arms and Disarms the vehicle. 
def arm(status):
	global arming
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		if arming != status:	
			arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			response = arming_cl(value = status)
			rospy.loginfo(response)
			arming = status
		else:
			print("Aleady set!")
	except rospy.ServiceException as e:
		print("Arming failed: %s" %e)

#Defines waypoint
#x_lat and y_long defined in meters
def create_waypoint(x,y):
	waypoint_clear_client()
	time.sleep(1)
	wl = []
	wp = Waypoint()   #Creates new instance of WayPoint
	wp.frame = 0 # PX4 Only supports Global frame
	wp.command = 22  #Takeoff command
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # HOLD time at WP
	wp.param2 = 0  # Acceptance radius, if inside WP count as reached
	wp.param3 = 0  # Pass Radius. If 0 go through WP
	wp.param4 = 0#float('nan')  # Yaw, 0 for our USV situation
	wp.x_lat = 0	#movement North/South
	wp.y_long = 0 #movement East/West
	wp.z_alt = 0
	wl.append(wp)

	wp2 = Waypoint()   #Creates new instance of WayPoint
	wp2.frame = 0 # PX4 Only supports Global frame
	wp2.command = 16  #Navigate to waypoint.
	wp2.is_current = False
	wp2.autocontinue = True
	wp2.param1 = 0  # HOLD time at WP
	wp2.param2 = 0  # Acceptance radius, if inside WP count as reached
	wp2.param3 = 0  # Pass Radius. If 0 go through WP
	wp2.param4 = 0 #float('nan')  # Yaw, 0 for our USV situation
	wp2.x_lat = x #movement North/South
	wp2.y_long = y #movement East/West
	wp2.z_alt = 0
	wl.append(wp2)


	wp3 = Waypoint()   #Creates new instance of WayPoint
	wp3.frame = 0 # PX4 Only supports Global frame
	wp3.command = 16  #Navigate to waypoint.
	wp3.is_current = False
	wp3.autocontinue = True
	wp3.param1 = 0  # HOLD time at WP
	wp3.param2 = 0  # Acceptance radius, if inside WP count as reached
	wp3.param3 = 0  # Pass Radius. If 0 go through WP
	wp3.param4 = 0 #float('nan')  # Yaw, 0 for our USV situation
	wp3.x_lat = x #movement North/South
	wp3.y_long = y #movement East/West
	wp3.z_alt = 0
	wl.append(wp3)

	print("This is X value: %s"%x)
	print("This is Y value: %s"%y)
	
	rospy.wait_for_service('mavros/mission/push')
	try:
		global wp_set
		serviceReq = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
		serviceRes = serviceReq(start_index=0, waypoints=wl)
		flag = serviceRes.success
		if flag == True:
			wp_set = True
			print('SUCCESS: PUSHING WP \n')
		elif flag == False:
			print('FAILURE: PUSHING WP \n')		
	except rospy.ServiceException as e:
		rospy.loginfo("setWayPoint failed: %s\n" %e)

def takeoff():
	rospy.wait_for_service('/mavros/cmd/takeoff')
	try:
		takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
   	 	response = takeoff_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    		rospy.loginfo(response)
		print ('Takeoff')
		print response
	except rospy.ServiceException, e:
    		print("Takeoff failed: %s" %e)

#Clears currently loaded waypoints
#Does not remove the currently active waypoint
def waypoint_clear_client():
	rospy.wait_for_service('mavros/mission/clear')
	global check
	try:
		response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
		check = False
		#print("Waypoint mission clear: %s"%response.call().success)		
		return response.call().success
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e
		return False

#
def get_orientation(msg):
	heading = round(int(msg)/10)*10
	global glob 
	glob = int(heading)

#
def compass_callback(msg):
	msg.data
	get_orientation(msg.data)

#
def set_wp(orientation):
	global check
	#rospy.wait_for_service('')
	if not check:
		key = glob
		x = dict_to_float(key,dict_x)
		y = dict_to_float(key,dict_y)
		x_lat = latitude(10)
		y_lon = longitude(y)
		create_waypoint(x_lat,y_lon)
		WP_x.write(str(x))	
		WP_x.write("\n")
		WP_y.write(str(y))
		WP_y.write("\n")
		check = True
###
def latitude(meters):
	earth = 6371000
	m = (1 / ((2 * pi / 360) * earth))
	new_latitude = lat + (meters * m)
	return new_latitude
###
def longitude(meters):
	earth = 6371000
	m = (1 / ((2 * pi / 360) * earth)) / 1000 # //1 meter in degree
	new_longitude = lon + (meters * m) / cos(lat * (pi / 180))
	return new_longitude
###
def gps_callback(gps):
	global lat
	global lon
	lat = gps.latitude
	lon = gps.longitude
###
def rc_callback(msg):
		radio = get_radio(msg.channels)
###
def get_radio(msg):
	global rc_input
	value = msg[1]

	if value > 2000:
		rc_input = True
	if value < 1000:
		rc_input = False
	else:
		rc_input = rc_input

### SAVES POSITION ###
def save_position():
	WP_lon.write(str(lon))	
	WP_lon.write("\n")
##### WRITE LAT AND LONG TO FILES IN 5 SEC INTERVALS #####
	WP_lat.write(str(lat))	
	WP_lat.write("\n")

def current_postion():
	WP_launch_lon.write(str(lon))	
##### WRITE LAT AND LONG TO FILES IN 5 SEC INTERVALS #####
	WP_launch_lat.write(str(lat))	

rospy.init_node('waypoint_node', anonymous=True)	#Initilizing ROS node with name
rate = rospy.Rate(10)
pub = rospy.Publisher('global',String,queue_size=10)	##
sub_compass = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, compass_callback) # subscribe to compass heading
sub_gps = rospy.Subscriber('mavros/global_position/global', NavSatFix, gps_callback) # subscribe to compass heading
sub_rc = rospy.Subscriber('mavros/rc/in', RCIn, rc_callback) 

while not rospy.is_shutdown():
	sub_gps
	sub_compass
	if rc_input == True:
		rospy.sleep(1)
		set_wp(glob)
		if wp_set == True:			
			change_mode(MODE_GUIDED)
			current_postion()
			wp_set = False

	if rc_input == True:
		save_position()
		print('POSITION SAVED')		
	if rc_input == False:
		waypoint_clear_client()
		print("WP CLEARED")
		change_mode(MODE_MANUAL)
	rospy.sleep(1)


