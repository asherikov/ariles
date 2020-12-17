VISITORS="jsonnet msgpack rapidjson pugixml octave namevalue graphviz"

clean:
	cd port; ${MAKE} clean

build:
	cd port; ${MAKE} build

checksum: # update
	cd port; ${MAKE} makesum

plist:
	rm -Rf port/pkg-plist*
	cd port; ${MAKE} makeplist > pkg-plist
	cd port; ${MAKE} makeplist > pkg-plist.tmp
	cd port; echo ${VISITORS} | tr ' ' '\n' | xargs -I {}  /bin/sh -c "grep -i {} pkg-plist.tmp > pkg-plist.{}"
	cd port; grep -i yaml pkg-plist.tmp > pkg-plist.yamlcpp
	cd port; echo ${VISITORS} | tr ' ' '\n' | xargs -I {}  sed -i '' "/{}/d" pkg-plist.tmp;
	cd port; sed -i '' "/yaml/d" pkg-plist.tmp
	cd port; sed -i '' "/makeplist/d" 	pkg-plist.tmp
	cd port; mv pkg-plist.tmp pkg-plist
	cd port; echo ${VISITORS} | tr ' ' '\n' | xargs -I {} \
		/bin/sh -c 'UPPER=`echo {} | tr "[:lower:]" "[:upper:]"`; sed "s/^/%%$${UPPER}%%/" pkg-plist.{} >> pkg-plist'
	cd port; sed "s/^/%%YAMLCPP%%/" pkg-plist.yamlcpp >> pkg-plist
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
