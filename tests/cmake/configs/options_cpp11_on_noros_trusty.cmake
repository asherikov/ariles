#- rapidjson-dev       # not found
#- libmsgpack-dev      # version is too old

set(ARILES_CPP_STANDARD         "c++11" CACHE STRING "")
set(ARILES_CPP_SANITIZERS       "ON"    CACHE STRING "")

set(ARILES_VISITOR_pugixml       "ON"    CACHE STRING "")
set(ARILES_VISITOR_rapidjson     "OFF"   CACHE STRING "")
set(ARILES_VISITOR_jsonnet       "OFF"   CACHE STRING "")
set(ARILES_VISITOR_msgpack       "OFF"   CACHE STRING "")
set(ARILES_VISITOR_yaml_cpp03    "OFF"   CACHE STRING "")
set(ARILES_VISITOR_yaml_cpp      "ON"    CACHE STRING "")
set(ARILES_VISITOR_ros           "OFF"   CACHE STRING "")
