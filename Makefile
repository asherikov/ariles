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

clean:
	rm -Rf build
	rm -Rf debian
	rm -Rf obj

release:
	catkin_prepare_release -t 'ros-' --version "${VERSION}" -y


#==============================================
# CI
#==============================================

# ROS
#----------------------------------------------

add-ros-repos:
	sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${UBUNTU_DISTRO} main\" > /etc/apt/sources.list.d/ros-latest.list"
	sh -c "apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
		|| apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
		|| apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116"
	apt-get update -qq
	apt-get install dpkg

install-ros:
	apt-get install -y ros-${ROS_DISTRO}-ros-base
	bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep init'

# Dependencies
install-deps:
	apt update
#	apt upgrade -y
	apt install -y \
        libeigen3-dev \
        octave \
        libyaml-cpp-dev
	apt install -y \
		python-bloom \
		devscripts \
		debhelper

ros-prerelease:
	sudo ${MAKE} add-ros-repos UBUNTU_DISTRO=${TRAVIS_UBUNTU_DISTRO}
	sudo apt update -y
	sudo apt install -y python3-ros-buildfarm
	mkdir -p /tmp/prerelease_job
	cd /tmp/prerelease_job && generate_prerelease_script.py \
		https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \
		${ROS_DISTRO} default ubuntu ${UBUNTU_DISTRO} amd64 \
		ariles_ros \
		--custom-branch ariles_ros:${BRANCH} \
		--level 0 \
		--output-dir ./
	cd /tmp/prerelease_job && ./prerelease.sh


# catkin
#----------------------------------------------

CATKIN_WORKING_DIR?=./build/catkin_workspace
PKG_PATH?=${CATKIN_WORKING_DIR}/src/catkin_ariles


catkin-build-old:
	mkdir -p ${PKG_PATH}
	cd ${CATKIN_WORKING_DIR}/src; catkin_init_workspace # old
	ls -1 | grep -v build | xargs cp -R -t ${PKG_PATH}
	cd ${CATKIN_WORKING_DIR}; catkin_make_isolated --pkg ariles_ros --cmake-args -DARILES_ROS_ENABLE_TESTS=ON --make-args all test # old

catkin-build-deb:
	cd ${PKG_PATH}; bloom-generate rosdebian --os-name ubuntu --ros-distro ${ROS_DISTRO} ./
	# disable installation of catkin stuff: setup scripts, etc.
	#cd ${PKG_PATH}; sed "s/dh_auto_configure --/dh_auto_configure -- -DCATKIN_BUILD_BINARY_PACKAGE=ON/" -i debian/rules
	cd ${PKG_PATH}; fakeroot debian/rules binary

catkin-test-deb:
	dpkg -i ${CATKIN_WORKING_DIR}/src/ros*ariles*.deb
	cd ${PKG_PATH}
	git checkout master
	bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash; ${MAKE} cmake_dependency'

catkin-test-old: install-deps
	${MAKE} catkin-build-old
	${MAKE} catkin-build-deb ROS_DISTRO=${ROS_DISTRO}
	${MAKE} catkin-test-deb ROS_DISTRO=${ROS_DISTRO}


#catkin-build-new: install-deps
#	mkdir -p ${CATKIN_WORKING_DIR}
#	mkdir -p ${CATKIN_WORKING_DIR}/src/catkin_ariles/
#	cd ${CATKIN_WORKING_DIR}; catkin init # new
#	ls -1 | grep -v build | xargs cp -R -t ${CATKIN_WORKING_DIR}/src/catkin_ariles
#	cd ${CATKIN_WORKING_DIR}; catkin build --verbose --summary # new


# docker
#----------------------------------------------
make-docker:
	docker pull ros:${ROS_DISTRO}-ros-base-${UBUNTU_DISTRO}
	docker run -ti ros:${ROS_DISTRO}-ros-base-${UBUNTU_DISTRO} \
		/bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
		&& git clone -b ${BRANCH} https://github.com/asherikov/ariles.git \
		&& cd ariles \
		&& make ${TARGET} ROS_DISTRO=${ROS_DISTRO} UBUNTU_DISTRO=${UBUNTU_DISTRO}"
