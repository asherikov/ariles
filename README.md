This is a proxy catkin package for Ariles serialization/configuration library,
see https://github.com/asherikov/ariles for more information.

Enabled data formats:
- yaml
- json
- ROS parameter server
- Octave script (output)
- array (flattened key-value pairs)

Note: ariles is not a catkin package, use `catkin(DEPENDS ariles2_core_ros)` in your
CMakeLists.txt, `find_package(catkin REQUIRED ariles2_core_ros)` still works.
