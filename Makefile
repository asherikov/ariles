clean:
	cd port; ${MAKE} clean

build:
	cd port; ${MAKE} build

checksum: # update
	cd port; ${MAKE} fetch
	cd port; ${MAKE} makesum

plist:
	rm -Rf port/pkg-plist*
	cd port; ${MAKE} makeplist > pkg-plist
	cd port; ${MAKE} makeplist > pkg-plist.tmp
	cd port; grep -i "jsonnet" 		pkg-plist.tmp > pkg-plist.jsonnet;
	cd port; grep -i "msgpack" 		pkg-plist.tmp > pkg-plist.msgpack;
	cd port; grep -i "rapidjson" 	pkg-plist.tmp > pkg-plist.rapidjson;
	cd port; grep -i "pugixml" 		pkg-plist.tmp > pkg-plist.pugixml;
	cd port; grep -i "octave" 		pkg-plist.tmp > pkg-plist.octave;
	cd port; grep -i "yaml" 		pkg-plist.tmp > pkg-plist.yamlcpp;\
		cp pkg-plist.yamlcpp 	pkg-plist.yamlcpp03;\
		sed -i "" 's/yaml_cpp/yaml_cpp03/g'  pkg-plist.yamlcpp03;\
		sed -i "" 's/yaml-cpp/yaml-cpp03/g'  pkg-plist.yamlcpp03
	cd port; \
	 	sed -i '' "/jsonnet/d" 		pkg-plist.tmp;\
		sed -i '' "/msgpack/d" 		pkg-plist.tmp;\
		sed -i '' "/rapidjson/d" 	pkg-plist.tmp;\
		sed -i '' "/pugixml/d" 		pkg-plist.tmp;\
		sed -i '' "/octave/d" 		pkg-plist.tmp;\
		sed -i '' "/yaml/d" 		pkg-plist.tmp;\
		sed -i '' "/makeplist/d" 	pkg-plist.tmp;
	cd port; mv pkg-plist.tmp pkg-plist
	cd port; sed "s/^/%%JSONNET%%/"		pkg-plist.jsonnet  	>>	pkg-plist
	cd port; sed "s/^/%%MSGPACK%%/"		pkg-plist.msgpack  	>> 	pkg-plist
	cd port; sed "s/^/%%RAPIDJSON%%/"	pkg-plist.rapidjson	>> 	pkg-plist
	cd port; sed "s/^/%%PUGIXML%%/"		pkg-plist.pugixml  	>> 	pkg-plist
	cd port; sed "s/^/%%OCTAVE%%/"		pkg-plist.octave   	>> 	pkg-plist
	cd port; sed "s/^/%%YAMLCPP%%/"		pkg-plist.yamlcpp 	>> 	pkg-plist
	cd port; sed "s/^/%%YAMLCPP03%%/"	pkg-plist.yamlcpp03	>> 	pkg-plist
	cd port; rm	 pkg-plist\.*

test: #user
	cd port; ${MAKE} stage
	cd port; ${MAKE} check-orphans
	cd port; ${MAKE} package
	cd port; ${MAKE} describe
	cd port; ${MAKE} clean
	${MAKE} portlint

portlint:
	# ports-mgmt/portlint
	cd port; portlint -A
	# ports-mgmt/porttools
	# port test

testroot:
	cd port; ${MAKE} install
	cd port; ${MAKE} deinstall
