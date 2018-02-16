MAKE_FLAGS?=-j5


CMAKE_DIR=./cmake/
BUILD_DIR=./build/
ROOT_DIR=../../


# Toolchains
TC?=generic

BRIDGES_MODE?=BUILD

# Build type as in CMAKE_BUILD_TYPE
TYPE?=Debug

TARGETS?=all

#----------------------------------------------
# Cleaning
#----------------------------------------------

clean:
	rm -Rf build;


#----------------------------------------------
# Generic targets
#----------------------------------------------

BUILD_SUBDIR=${BUILD_DIR}/${TC}-${TYPE}-BRIDGES_${BRIDGES_MODE}

build:
	mkdir -p ${BUILD_SUBDIR};
	cd ${BUILD_SUBDIR}; cmake 	-DCMAKE_BUILD_TYPE=${TYPE} \
								-DCMAKE_TOOLCHAIN_FILE=${CMAKE_DIR}/toolchain_${TC}.cmake\
								-DARILES_BRIDGES_DEFAULT_MODE=${BRIDGES_MODE}\
								${EXTRA_CMAKE_PARAM} \
								${ROOT_DIR};
	cd ${BUILD_SUBDIR}; ${MAKE} ${MAKE_FLAGS} ${TARGETS}

build-tests: build
	cd ${BUILD_SUBDIR}; ${MAKE} ${MAKE_FLAGS} test

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

test: debug-all-tests

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
# checks
#----------------------------------------------

check-build: clean
	${MAKE} build-tests TC=${TC} TYPE=Debug BRIDGES_MODE=ON TARGETS="${TARGETS}" 	EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
	${MAKE} build-tests TC=${TC} TYPE=Debug BRIDGES_MODE=BUILD TARGETS="${TARGETS}" EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
#	${MAKE} build-tests TC=gcc TYPE=Debug BRIDGES_MODE=ON TARGETS="${TARGETS}" 		EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
#	${MAKE} build-tests TC=gcc TYPE=Debug BRIDGES_MODE=BUILD TARGETS="${TARGETS}" 	EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM}"
#	${MAKE} build-tests TC=gcc TYPE=Debug BRIDGES_MODE=ON TARGETS="${TARGETS}" 		EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM} -DARILES_CXX_FLAGS=-D_GLIBCXX_USE_CXX11_ABI=0"

check: check-build


#----------------------------------------------
# other
#----------------------------------------------

update:
	git submodule update

.PHONY: clean cmake build
