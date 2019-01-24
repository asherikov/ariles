BUILDDIR?=build
MAKE_FLAGS?=-j1

default:
	@grep -v "^	" Makefile | grep -v "^$$"

build: clean
	mkdir -p ${BUILDDIR}
	cd ${BUILDDIR}; cmake ..
	cd ${BUILDDIR}; ${MAKE} ${MAKE_FLAGS}

install:
	cd ${BUILDDIR}; ${MAKE} ${MAKE_FLAGS} install

test: clean
	mkdir -p ${BUILDDIR}/cmake_dependency_test
	cd ${BUILDDIR}/cmake_dependency_test; cmake ../../test/cmake_dependency/
	cd ${BUILDDIR}/cmake_dependency_test; ${MAKE} ${MAKE_FLAGS}

clean:
	rm -Rf ${BUILDDIR}

.PHONY: build
