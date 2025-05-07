Gz_projects files has all the rover models integrated with the simulations 

Run the Rock_world.world file in gz to get the simulation environment 

source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /world/my_world/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/diffbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  "/world/my_world/model/diffbot/link/base_link/sensor/rgb_camera/image@sensor_msgs/msg/Image@gz.msgs.Image" \
  "/world/my_world/model/diffbot/link/base_link/sensor/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
  "/world/my_world/model/diffbot/link/base_link/sensor/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image" \
  "/world/my_world/model/diffbot/link/base_link/sensor/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
  --ros-args \
    -r /world/my_world/clock:=/clock \
    -r /model/diffbot/odometry:=/odom
