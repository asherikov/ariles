help:
	-@grep --color -Ev "(^	)|(^$$)" Makefile
	-@grep --color -Ev "(^	)|(^$$)" make/*.mk


# release
#----------------------------------------------

update_version:
	sed -i -e "s=\(project([ a-zA-Z0-9_-]* VERSION\) [0-9]*\.[0-9]*\.[0-9]*)=\1 ${VERSION})=" */CMakeLists.txt
	#done automatically
	sed -i -e "s=<version>[0-9]*\.[0-9]*\.[0-9]*</version>=<version>${VERSION}</version>=" */package.xml

