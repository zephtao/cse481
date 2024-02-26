String shape
float64 start_x
float64 start_y
#TODO: not sure how to communicate shape dimensions
	option 1: use geometry_msgs/msg/Polygon, which containts Point32[] where 1st and last point are connected
	option 2: use shape dimensions or custom class and send message
