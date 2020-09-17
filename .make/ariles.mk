BUILD_DIR?=build
MAKE_FLAGS?=-j5
VERSION?="XXX__version_not_set__XXX"


REPO=https://github.com/asherikov/ariles.git
DEPENDENCY_PATH=./demo
DEBIAN_SYSTEM_DEPENDENCIES=libeigen3-dev octave libyaml-cpp-dev rapidjson-dev


update:
	git fetch --all
	git rm --ignore-unmatch -rf ariles
	git read-tree --prefix=ariles -u ${VERSION}
	${MAKE} cleanup

update_head:
	${MAKE} update VERSION="origin/master"


# clean
#----------------------------------------------

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
	rm -Rf obj*

.PHONY: build cmake tests