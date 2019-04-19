MAKE_FLAGS?=-j5


CMAKE_DIR=./cmake/
BUILD_DIR=./build/
ROOT_DIR=../../


# Toolchains
TC?=generic

OPTIONS?=default

# Build type as in CMAKE_BUILD_TYPE
TYPE?=Debug

TARGETS?=all

ARGS?=

DEB_TARGET?=xenial


#----------------------------------------------
# Cleaning
#----------------------------------------------

clean:
	rm -Rf build;
	rm -Rf include/ariles/internal/cpput_*.h
	git submodule update --init doc/dox/; cd doc/dox/; git clean -f; git reset --hard


#----------------------------------------------
# Generic targets
#----------------------------------------------

BUILD_SUBDIR=${BUILD_DIR}/${TC}-${TYPE}-OPTIONS_${OPTIONS}

build:
	mkdir -p ${BUILD_SUBDIR};
	cd ${BUILD_SUBDIR}; cmake 	-C ${ROOT_DIR}/tests/cmake_configs/options_${OPTIONS}.cmake\
								-DCMAKE_BUILD_TYPE=${TYPE} \
								-DCMAKE_TOOLCHAIN_FILE=${CMAKE_DIR}/toolchain_${TC}.cmake\
								${EXTRA_CMAKE_PARAM} \
								${ROOT_DIR};
	cd ${BUILD_SUBDIR}; ${MAKE} ${MAKE_FLAGS} ${TARGETS}

build-tests: build
	cd ${BUILD_SUBDIR}; ctest ${ARGS}
#	cd ${BUILD_SUBDIR}; ${MAKE} ${MAKE_FLAGS} test ${ARGS}

# -------

FETCH_DIR=${BUILD_DIR}/fetch

fetch:
	mkdir -p ${FETCH_DIR};
	cd ${FETCH_DIR}; cmake ${ROOT_DIR};
	cd ${FETCH_DIR}; ${MAKE} fetch-bridges;


#----------------------------------------------
# debug mode (all)
# Build & test
#----------------------------------------------

debug-all:
	${MAKE} build TC=${TC} TYPE=Debug TARGETS="${TARGETS}"

debug-all-tests:
	${MAKE} build-tests TC=${TC} TYPE=Debug TARGETS="${TARGETS}"

all: debug-all

debug: debug-all


#----------------------------------------------
# release mode
# Build & test
#----------------------------------------------

release-all:
	${MAKE} build TC=${TC} TYPE=Release TARGETS="${TARGETS}"

release-all-tests: release-all
	${MAKE} build-tests TC=${TC} TYPE=Release TARGETS="${TARGETS}"

release: release-all


#----------------------------------------------
# Debian packages
#----------------------------------------------

deb: clean
	${MAKE} build TC=${TC} TYPE=Release OPTIONS=deb_packages_${DEB_TARGET} TARGETS="pkg_deb" \
		EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM} -DDEB_BUILDPACKAGE_FLAGS='-d'"

deb-build: clean
	${MAKE} build TC=${TC} TYPE=Release OPTIONS=deb_packages_${DEB_TARGET} TARGETS="pkg_deb" \
		EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM} -DDEB_BUILDPACKAGE_FLAGS='-us;-uc'"
	cd build/generic-Release-OPTIONS_deb_packages_${DEB_TARGET}/Debian/${DEB_TARGET}/ariles-*-source; \
		dpkg-buildpackage -us -uc -F

deb-install:
	sudo dpkg -i build/generic-Release-OPTIONS_deb_packages_${DEB_TARGET}/Debian/${DEB_TARGET}/ariles-*.deb

deb-uninstall:
	dpkg --get-selections ariles* | awk '{print $1}' | xargs sudo dpkg -P

cmake_dependency: clean
	mkdir -p build/cmake_dependency_test
	cd build/cmake_dependency_test; cmake ../../tests/cmake_dependency/ -DARILES_COMPONENTS="core;ros;yaml-cpp"
	cd build/cmake_dependency_test; ${MAKE} ${MAKE_FLAGS}

ppa-upload:
	cd build/generic-Release-OPTIONS_deb_packages_trusty/Debian/trusty/; \
		ftp -au ppa.launchpad.net:~asherikov/ubuntu/ppa/ \
			ariles_*~${DEB_TARGET}.dsc \
			ariles_*~${DEB_TARGET}.tar.xz \
			ariles_*~${DEB_TARGET}_source.buildinfo \
			ariles_*~${DEB_TARGET}_source.changes


#----------------------------------------------
# catkin
#----------------------------------------------

CATKIN_WORKING_DIR?=./build/catkin_workspace

catkin-build-old: clean
	mkdir -p ${CATKIN_WORKING_DIR}
	mkdir -p ${CATKIN_WORKING_DIR}/src/catkin_ariles/
	cd ${CATKIN_WORKING_DIR}/src; catkin_init_workspace # old
	ls -1 | grep -v build | xargs cp -R -t ${CATKIN_WORKING_DIR}/src/catkin_ariles
	cd ${CATKIN_WORKING_DIR}; catkin_make_isolated # old

catkin-build-new: clean
	mkdir -p ${CATKIN_WORKING_DIR}
	mkdir -p ${CATKIN_WORKING_DIR}/src/catkin_ariles/
	cd ${CATKIN_WORKING_DIR}; catkin init # new
	ls -1 | grep -v build | xargs cp -R -t ${CATKIN_WORKING_DIR}/src/catkin_ariles
	cd ${CATKIN_WORKING_DIR}; catkin build --verbose --summary # new


#----------------------------------------------
# checks
#----------------------------------------------


test-ros: clean
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=default TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=ros TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"


test-noros: clean
	${MAKE} build TC=${TC} TYPE=Debug OPTIONS=no_core TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=cpp11_on_noros TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=cpp11_on_noros_sloppy TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=cpp03_on_noros TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=cpp11_build_noros TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=cpp03_build_noros TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"

test-cmake:
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=conflict TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"

#----------------------------------------------
# other
#----------------------------------------------

addutils:
	git remote add cmakeut https://github.com/asherikov/cmakeut
	git remote add cpput https://github.com/asherikov/cpput

updateutils:
	git fetch --all
	git show remotes/cmakeut/master:cmake/FindEigen3.cmake > cmake/FindEigen3.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_add_external_git_project.cmake    > cmake/cmakeut_add_external_git_project.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_compiler_flags.cmake              > cmake/cmakeut_compiler_flags.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_copy_dir_if_exists.cmake          > cmake/cmakeut_copy_dir_if_exists.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_copy_file_if_exists.cmake         > cmake/cmakeut_copy_file_if_exists.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_detect_func_macro.cmake           > cmake/cmakeut_detect_func_macro.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_dump_variables.cmake              > cmake/cmakeut_dump_variables.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_list_filenames.cmake              > cmake/cmakeut_list_filenames.cmake
	git rm --ignore-unmatch -rf cpput
	git read-tree --prefix=cpput -u cpput/master


update:
	git submodule update

dox:
	cd doc; doxygen

install-ros:
	sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${UBUNTU_DISTRO} main\" > /etc/apt/sources.list.d/ros-latest.list"
	sh -c "apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
		|| apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
		|| apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116"
	apt-get update -qq
	apt-get install dpkg
	apt-get install -y ros-${ROS_DISTRO}-ros-base
	bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep init'

.PHONY: clean cmake build
