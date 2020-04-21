/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#ifdef ARILES_VISITOR_yaml_cpp03
#    include "ariles/visitors/yaml_cpp03.h"
#endif

#ifdef ARILES_VISITOR_yaml_cpp
#    include "ariles/visitors/yaml_cpp.h"
#endif

#ifdef ARILES_VISITOR_msgpack
#    include "ariles/visitors/msgpack.h"
#endif

#ifdef ARILES_VISITOR_ros
#    include "ariles/visitors/ros.h"
#endif

#ifdef ARILES_VISITOR_jsonnet
#    include "ariles/visitors/jsonnet.h"
#endif

#ifdef ARILES_VISITOR_rapidjson
#    include "ariles/visitors/rapidjson.h"
#endif

#ifdef ARILES_VISITOR_pugixml
#    include "ariles/visitors/pugixml.h"
#endif

// Do not have Reader and therefore are excluded from most of the tests
//
//#ifdef ARILES_VISITOR_octave
//#include "ariles/visitors/octave.h"
//#endif
//
//#ifdef ARILES_VISITOR_array
//#include "ariles/visitors/array.h"
//#endif
