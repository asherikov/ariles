APT_INSTALL?=env DEBIAN_FRONTEND=noninteractive apt --yes --no-install-recommends install
MAKE_FLAGS?=-j14


CMAKE_DIR=./cmake/
BUILD_DIR=./build/
INSTALL_DIR=./install/
ROOT_DIR=../../


# Toolchains
TC?=generic

OPTIONS?=default

# Build type as in CMAKE_BUILD_TYPE
TYPE?=Debug

TARGETS?=all

ARGS?=

DEB_TARGET?=xenial

SUPP_PATH=../../../qa/sanitizers/
# new_delete_type_mismatch=0 ROS2 issue -> https://github.com/ros2/rclcpp/issues/2220
TEST_ENV=UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1:suppressions=${SUPP_PATH}/undefined.supp \
		ASAN_OPTIONS=new_delete_type_mismatch=0:suppressions=${SUPP_PATH}/address.supp \
		LSAN_OPTIONS=suppressions=${SUPP_PATH}/leak.supp

PKG_NAME=ariles2

FIND_ARILES_SOURCES=find ./extra_* ./tests/ ./include/ -iname "*.h" -or -iname "*.cpp"


#----------------------------------------------
# Cleaning
#----------------------------------------------

clean:
	rm -Rf build;
	rm -Rf include/${PKG_NAME}/internal/cpput_*.h
	#git submodule update --init doc/dox/; cd doc/dox/; git clean -f; git reset --hard


#----------------------------------------------
# Generic targets
#----------------------------------------------

BUILD_SUBDIR=${BUILD_DIR}/${TC}-${TYPE}-OPTIONS_${OPTIONS}
INSTALL_SUBDIR=${INSTALL_DIR}/${TC}-${TYPE}-OPTIONS_${OPTIONS}

build:
	mkdir -p ${BUILD_SUBDIR};
	cd ${BUILD_SUBDIR}; cmake 	-C ${ROOT_DIR}/cmake/options_${OPTIONS}.cmake\
								-DCMAKE_BUILD_TYPE=${TYPE} \
								-DCMAKE_TOOLCHAIN_FILE=${CMAKE_DIR}/toolchain_${TC}.cmake \
								-DCMAKE_INSTALL_PREFIX=${INSTALL_SUBDIR} \
								${EXTRA_CMAKE_PARAM} \
								${ROOT_DIR};
	cd ${BUILD_SUBDIR}; ${MAKE} ${MAKE_FLAGS} ${TARGETS}

build-tests:
	${MAKE} build EXTRA_CMAKE_PARAM="-DARILES_CPP_SANITIZERS=ON -DARILES_BUILD_REGRESSION_TESTS=ON ${EXTRA_CMAKE_PARAM}"
	cd ${BUILD_SUBDIR}; env ${TEST_ENV} ctest ${ARGS}
#	cd ${BUILD_SUBDIR}; ${MAKE} ${MAKE_FLAGS} test ${ARGS}

# -------

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
	cd build/generic-Release-OPTIONS_deb_packages_${DEB_TARGET}/Debian/${DEB_TARGET}/${PKG_NAME}-*-source; \
		dpkg-buildpackage -us -uc -F

deb-install:
	sudo dpkg -i build/generic-Release-OPTIONS_deb_packages_${DEB_TARGET}/Debian/${DEB_TARGET}/${PKG_NAME}-*.deb

deb-uninstall:
	dpkg --get-selections ${PKG_NAME}* | awk '{print $1}' | xargs sudo dpkg -P

deb-cloudsmith:
	ls build/generic-Release-OPTIONS_deb_packages_${DEB_TARGET}/Debian/${DEB_TARGET}/${PKG_NAME}-*.deb \
		| xargs --no-run-if-empty -I {} cloudsmith push deb asherikov-aV7/all/ubuntu/${DEB_TARGET} {}

cmake_dependency: clean
	mkdir -p build/cmake_dependency_test
	cd build/cmake_dependency_test; cmake ../../tests/dependency/ -DARILES_COMPONENTS="rosparam;yaml-cpp;octave"
	cd build/cmake_dependency_test; ${MAKE} ${MAKE_FLAGS}

#ppa-upload:
#	cd build/generic-Release-OPTIONS_deb_packages_trusty/Debian/trusty/; \
#		ftp -au ppa.launchpad.net:~asherikov/ubuntu/ppa/ \
#			${PKG_NAME}_*~${DEB_TARGET}.dsc \
#			${PKG_NAME}_*~${DEB_TARGET}.tar.xz \
#			${PKG_NAME}_*~${DEB_TARGET}_source.buildinfo \
#			${PKG_NAME}_*~${DEB_TARGET}_source.changes


#----------------------------------------------
# checks
#----------------------------------------------


test-ros: clean
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=default TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=ros TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"


test-ros2: clean
	#${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=default TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=ros2 TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"


test-noros: clean
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=noros TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} clangcheck SCANBUILD=scan-build20 OPTIONS=noros_tidy
	${MAKE} cppcheck
	${MAKE} spell


#----------------------------------------------
# other
#----------------------------------------------

addutils:
	-git remote add --no-tags cmakeut https://github.com/asherikov/cmakeut
	-git remote add --no-tags cpput https://github.com/asherikov/cpput

updateutils: addutils
	git fetch --all
	git show remotes/cmakeut/master:cmake/FindEigen3.cmake             > cmake/FindEigen3.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_compiler_flags.cmake > cmake/cmakeut_compiler_flags.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_dump_variables.cmake > cmake/cmakeut_dump_variables.cmake
	git show remotes/cmakeut/master:cmake/cmakeut_list_filenames.cmake > cmake/cmakeut_list_filenames.cmake
	#
	git show remotes/cpput/master:include/cpput/exception.h  > include/ariles2/internal/exception.h
	git show remotes/cpput/master:include/cpput/visibility.h > include/ariles2/internal/visibility.h
	git show remotes/cpput/master:include/cpput/misc.h       > include/ariles2/internal/misc.h
	git show remotes/cpput/master:include/cpput/trace.h      > include/ariles2/internal/trace.h
	git show remotes/cpput/master:include/cpput/concat.h     > include/ariles2/internal/concat.h


update:
	git submodule update


doxclean:
	cd doc/dox; git fetch --all; git checkout gh-pages; git pull
	rm -Rf ./doc/dox/2

dox: doxclean clean
	cd doc; doxygen


install-ros:
	wget -qO- https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc
	${MAKE} install-ros-${ROS_DISTRO}
	${APT_INSTALL} dpkg
	${APT_INSTALL} python3-rosdep build-essential
	bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash; rosdep init'
	rosdep update --rosdistro ${ROS_DISTRO}

install-ros-noetic:
	sh -c 'test -f /etc/apt/sources.list.d/ros-latest.list \
        || (echo "deb http://packages.ros.org/ros/ubuntu ${UBUNTU_DISTRO} main" > /etc/apt/sources.list.d/ros-latest.list)'
	apt update -qq
	${APT_INSTALL} ros-${ROS_DISTRO}-ros-base python3-rosinstall python3-rosinstall-generator

install-ros2-common:
	sh -c 'test -f /etc/apt/sources.list.d/ros2-latest.list \
        || (echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main ${UBUNTU_DISTRO} main" > /etc/apt/sources.list.d/ros2-latest.list)'
	apt update -qq
	${APT_INSTALL} ros-${ROS_DISTRO}-rclcpp

install-ros-humble: install-ros2-common
	${APT_INSTALL} python3-rosinstall python3-rosinstall-generator

install-ros-jazzy: install-ros2-common
	#


install-deps:
	${APT_INSTALL} cmake libboost-all-dev libeigen3-dev
	${APT_INSTALL} octave libpugixml-dev libyaml-cpp-dev rapidjson-dev libmsgpack-dev graphviz
	${APT_INSTALL} libprotobuf-dev protobuf-compiler
	${APT_INSTALL} libjsonnet-dev libjsonnet0


format:
	${FIND_ARILES_SOURCES} | grep -v "better_enum.h" | xargs clang-format-15 -verbose -i

cppcheck:
	# --inconclusive
	# false positive: constStatement, unsignedLessThanZero
	cppcheck \
		./ \
		--inline-suppr \
		--relative-paths \
		--quiet --verbose --force \
		--template='[{file}:{line}]  {severity}  {id}  {message}' \
		--language=c++ --std=c++11 \
	 	--enable=warning \
		--enable=style \
		--enable=performance \
		--enable=portability \
		--suppress=uninitMemberVar \
		--suppress=syntaxError \
		--suppress=useInitializationList \
		--suppress=functionStatic \
		--suppress=unknownMacro \
		--suppress=constStatement \
		--suppress=unsignedLessThanZero \
		--suppress=duplInheritedMember \
		-i build \
		-i tests/api_v2/regression_test_230.cpp \
		{} \
	3>&1 1>&2 2>&3 | tee cppcheck.err
	test 0 -eq `cat cppcheck.err | wc -l && rm cppcheck.err`
	# check headers
	find ./ -type f -iname '*.hpp' -or -iname "*.h" \
		| grep -v "better_enum.h" \
		| grep -v ".*build/.*" \
		| xargs --max-procs=1 --no-run-if-empty -I {} \
	cppcheck \
		--inline-suppr \
		--relative-paths \
		--quiet --verbose --force \
		--template='[{file}:{line}]  {severity}  {id}  {message}' \
		--language=c++ --std=c++11 \
	 	--enable=warning \
		--enable=style \
		--enable=performance \
		--enable=portability \
		--suppress=uninitMemberVar \
		--suppress=syntaxError \
		--suppress=useInitializationList \
		--suppress=functionStatic \
		--suppress=unknownMacro \
		--suppress=constStatement \
		--suppress=unsignedLessThanZero \
		--suppress=duplInheritedMember \
		--suppress=unreadVariable \
		--suppress=unusedStructMember \
		-i tests/api_v2/regression_test_230.cpp \
		{} \
	3>&1 1>&2 2>&3 | tee cppcheck.err
	test 0 -eq `cat cppcheck.err | wc -l && rm cppcheck.err`

#
# make clangcheck SCANBUILD=scan-build-9 OPTIONS=ros
# make clangcheck SCANBUILD=scan-build11 OPTIONS=cpp11_on_noros_tidy
#
clangcheck:
	${SCANBUILD} \
		-o build/scanbuild_results \
		--status-bugs \
		--exclude ./build \
		--exclude /usr/include/ \
		--exclude /usr/local/include/ \
		--exclude /usr/src/ \
		--exclude /opt/ros/ \
		-enable-checker core.CallAndMessage \
		-enable-checker core.DivideZero \
		-enable-checker core.DynamicTypePropagation \
		-enable-checker core.NonNullParamChecker \
		-enable-checker core.NullDereference \
		-enable-checker core.StackAddressEscape \
		-enable-checker core.UndefinedBinaryOperatorResult \
		-enable-checker core.VLASize \
		-enable-checker core.uninitialized.ArraySubscript \
		-enable-checker core.uninitialized.Assign \
		-enable-checker core.uninitialized.Branch \
		-enable-checker core.uninitialized.CapturedBlockVariable \
		-enable-checker core.uninitialized.UndefReturn \
		-enable-checker cplusplus.InnerPointer \
		-enable-checker cplusplus.Move \
		-enable-checker cplusplus.NewDeleteLeaks \
		-enable-checker deadcode.DeadStores \
		-enable-checker nullability.NullPassedToNonnull \
		-enable-checker nullability.NullReturnedFromNonnull \
		-enable-checker nullability.NullableDereferenced \
		-enable-checker nullability.NullablePassedToNonnull \
		-enable-checker nullability.NullableReturnedFromNonnull \
		-enable-checker optin.cplusplus.UninitializedObject \
		-enable-checker optin.mpi.MPI-Checker \
		-enable-checker optin.performance.GCDAntipattern \
		-enable-checker optin.performance.Padding \
		-enable-checker optin.portability.UnixAPI \
		-enable-checker security.FloatLoopCounter \
		-enable-checker security.insecureAPI.DeprecatedOrUnsafeBufferHandling \
		-enable-checker security.insecureAPI.UncheckedReturn \
		-enable-checker security.insecureAPI.getpw \
		-enable-checker security.insecureAPI.gets \
		-enable-checker security.insecureAPI.mkstemp \
		-enable-checker security.insecureAPI.mktemp \
		-enable-checker security.insecureAPI.vfork \
		-enable-checker unix.API \
		-enable-checker unix.Malloc \
		-enable-checker unix.MallocSizeof \
		-enable-checker unix.MismatchedDeallocator \
		-enable-checker unix.Vfork \
		-enable-checker unix.cstring.BadSizeArg \
		-enable-checker unix.cstring.NullArg \
		-enable-checker valist.CopyToSelf \
		-enable-checker valist.Uninitialized \
		-enable-checker valist.Unterminated \
		${MAKE} build TC=${TC} TYPE=${TYPE} OPTIONS=${OPTIONS} TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="-DARILES_BUILD_REGRESSION_TESTS=ON -DBUILD_SHARED_LIBS=OFF ${EXTRA_CMAKE_PARAM}"
#		-enable-checker cplusplus.NewDelete
#		-enable-checker optin.cplusplus.VirtualCall

spell_interactive:
	${MAKE} spell SPELL_XARGS_ARG=-o

# https://github.com/myint/scspell
spell:
	${FIND_ARILES_SOURCES} \
		| grep -v "./tests/common/better_enum.h" \
		| grep -v "./extra_visitors/rapidjson/src/istreamwrapper.h" \
		| xargs ${SPELL_XARGS_ARG} scspell --use-builtin-base-dict --override-dictionary ./qa/scspell.dict

.PHONY: clean cmake build
