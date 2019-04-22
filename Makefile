VERSION?="XXX__version_not_set__XXX"

update:
	git fetch --all
	git rm --ignore-unmatch -rf ariles
	git read-tree --prefix=ariles -u origin/master
	${MAKE} cleanup

cleanup:
	git rm -rf --ignore-unmatch ariles/bridges/jsonnet/jsonnet
	git rm -rf --ignore-unmatch ariles/bridges/msgpack/msgpack-c
	git rm -rf --ignore-unmatch ariles/bridges/pugixml/pugixml
	git rm -rf --ignore-unmatch ariles/bridges/rapidjson/rapidjson
	git rm -rf --ignore-unmatch ariles/bridges/yaml_cpp/yaml-cpp
	git rm -rf --ignore-unmatch ariles/bridges/yaml_cpp03/yaml-cpp
	git rm -rf --ignore-unmatch ariles/.gitmodules
	git rm -rf --ignore-unmatch ariles/tests/catkin_package/
	git rm -rf --ignore-unmatch ariles/doc/dox

release:
	catkin_prepare_release -t 'ros-' --version "${VERSION}" -y


#==============================================
# CI
#==============================================

# ROS
#----------------------------------------------

install-ros:
	sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${UBUNTU_DISTRO} main\" > /etc/apt/sources.list.d/ros-latest.list"
	sh -c "apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
		|| apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
		|| apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116"
	apt-get update -qq
	apt-get install dpkg
	apt-get install -y ros-${ROS_DISTRO}-ros-base
	bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep init'


# catkin
#----------------------------------------------

CATKIN_WORKING_DIR?=./build/catkin_workspace

catkin-build-old:
	mkdir -p ${CATKIN_WORKING_DIR}
	mkdir -p ${CATKIN_WORKING_DIR}/src/catkin_ariles/
	cd ${CATKIN_WORKING_DIR}/src; catkin_init_workspace # old
	ls -1 | grep -v build | xargs cp -R -t ${CATKIN_WORKING_DIR}/src/catkin_ariles
	cd ${CATKIN_WORKING_DIR}; catkin_make_isolated # old

catkin-build-new:
	mkdir -p ${CATKIN_WORKING_DIR}
	mkdir -p ${CATKIN_WORKING_DIR}/src/catkin_ariles/
	cd ${CATKIN_WORKING_DIR}; catkin init # new
	ls -1 | grep -v build | xargs cp -R -t ${CATKIN_WORKING_DIR}/src/catkin_ariles
	cd ${CATKIN_WORKING_DIR}; catkin build --verbose --summary # new
