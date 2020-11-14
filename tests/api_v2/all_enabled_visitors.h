/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#ifdef ARILES_VISITOR_yaml_cpp
#    include <ariles2/visitors/yaml_cpp.h>
#endif

#ifdef ARILES_VISITOR_msgpack
#    include <ariles2/visitors/msgpack.h>
#endif

#ifdef ARILES_VISITOR_rosparam
#    include <ariles2/visitors/rosparam.h>
#endif

#ifdef ARILES_VISITOR_jsonnet
#    include <ariles2/visitors/jsonnet.h>
#endif

#ifdef ARILES_VISITOR_rapidjson
#    include <ariles2/visitors/rapidjson.h>
#endif

#ifdef ARILES_VISITOR_pugixml
#    include <ariles2/visitors/pugixml.h>
#endif

// Do not have Reader and therefore are excluded from most of the tests
//
//#ifdef ARILES_VISITOR_octave
//#include <ariles2/visitors/octave.h>
//#endif
//
//#ifdef ARILES_VISITOR_namevalue
//#include <ariles2/visitors/namevalue.h>
//#endif
//
//#ifdef ARILES_VISITOR_graphviz
//#include <ariles2/visitors/namevalue.h>
//#endif
