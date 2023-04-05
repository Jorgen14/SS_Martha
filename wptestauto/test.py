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

print dict_x
