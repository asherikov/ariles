This is a proxy catkin package for Ariles serialization/configuration library,
see https://github.com/asherikov/ariles for more information.

Available data formats:
- yaml
- json
- ROS parameter server
- ROS2 parameters
- Octave script (output)
- array (flattened key-value pairs)

Note: ariles is not a catkin package, use `catkin(DEPENDS ariles2_core_catkin)`
in your CMakeLists.txt, `find_package(catkin REQUIRED ariles2_core_catkin)`
still works.
