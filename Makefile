BUILD_DIR?=build
MAKE_FLAGS?=-j1

default:
	@grep -v "^	" Makefile | grep -v "^$$"

build: clean
	mkdir -p ${BUILD_DIR}
	cd ${BUILD_DIR}; cmake ..
	cd ${BUILD_DIR}; ${MAKE} ${MAKE_FLAGS}

install:
	cd ${BUILD_DIR}; ${MAKE} ${MAKE_FLAGS} install

test: clean
	mkdir -p ${BUILD_DIR}/cmake_dependency_test
	cd ${BUILD_DIR}/cmake_dependency_test; cmake ../../test/cmake_dependency/
	cd ${BUILD_DIR}/cmake_dependency_test; ${MAKE} ${MAKE_FLAGS}

clean:
	rm -Rf ${BUILD_DIR}

.PHONY: build
