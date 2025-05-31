# run the gps driver node
ros2 run gps_driver_python gps_node

# add a gps_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map gps_link



