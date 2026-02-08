#set(ARILES_CPP_SANITIZERS       "ON"    CACHE STRING "")
set(ARILES_CPP_CLANG_TIDY       "ON"    CACHE STRING "")

set(ARILES_VISITOR_graphviz     "ON"    CACHE STRING "")
set(ARILES_VISITOR_pugixml      "ON"    CACHE STRING "")
# disabled due to due to https://github.com/Tencent/rapidjson/issues/2347
#set(ARILES_VISITOR_rapidjson    "ON"    CACHE STRING "")
set(ARILES_VISITOR_nlohmann_json "ON"   CACHE STRING "")
set(ARILES_VISITOR_jsonnet      "ON"    CACHE STRING "")
set(ARILES_VISITOR_msgpack      "ON"    CACHE STRING "")
set(ARILES_VISITOR_yaml_cpp     "ON"    CACHE STRING "")
set(ARILES_VISITOR_rosparam     "OFF"   CACHE STRING "")
set(ARILES_VISITOR_ros2param    "OFF"   CACHE STRING "")
