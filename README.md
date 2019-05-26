This is a proxy catkin package for Ariles serialization/configuration library,
see https://github.com/asherikov/ariles for more information.

Enabled configuration formats:
 - yaml
 - ROS parameter server
 - Octave script (output)

Note: ariles is not a catkin package, use `catkin(DEPENDS ariles_ros)` in your
CMakeLists.txt, `find_package(catkin REQUIRED ariles_ros)` still works.
