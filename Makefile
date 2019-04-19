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
