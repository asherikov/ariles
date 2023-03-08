help:
	-@grep --color -Ev "(^	)|(^$$)" Makefile
	-@grep --color -Ev "(^	)|(^$$)" GNUmakefile
	-@grep --color -Ev "(^	)|(^$$)" make/Makefile*


# release
#----------------------------------------------

update_version:
	sed -i -e "s=\(project([ a-zA-Z0-9_-]* VERSION\) [0-9]*\.[0-9]*\.[0-9]*)=\1 ${VERSION})=" ariles2_*/CMakeLists.txt
	#done automatically
	sed -i -e "s=<version>[0-9]*\.[0-9]*\.[0-9]*</version>=<version>${VERSION}</version>=" ariles2_*/package.xml

