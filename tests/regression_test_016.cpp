/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#define ARILES_TESTS_BOOST_UTF_DISABLED

#ifdef ARILES_BRIDGE_yaml_cpp03
#include "ariles/bridges/yaml_cpp03.h"
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
#include "ariles/bridges/yaml_cpp.h"
#endif

#ifdef ARILES_BRIDGE_msgpack
#include "ariles/bridges/msgpack.h"
#endif

#ifdef ARILES_BRIDGE_ros
#include "ariles/bridges/ros.h"
#endif

#ifdef ARILES_BRIDGE_rapidjson
#include "ariles/bridges/rapidjson.h"
#endif

#ifdef ARILES_BRIDGE_pugixml
#include "ariles/bridges/pugixml.h"
#endif

#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/complex_auto_declare.h"
#include "types/complex_base.h"
#include "types/complex_verbose.h"
#include "types/empty.h"
#include "types/enum.h"
#include "types/finalize.h"
#include "types/inheritance.h"
#include "types/minimal.h"
#include "types/no_setdefaults.h"
#include "types/pointers.h"
#include "types/simple_auto_declare.h"
#include "types/simple_no_auto_id.h"
#include "types/simple_no_constructors.h"
#include "types/simple_verbose.h"
#include "types/special_floats.h"
#include "types/strictness.h"

int main()
{
    ConfigurableComplex             a;
    ConfigurableComplexVerbose      b;
    ConfigurableEmpty               c;
    ConfigurableFinalize            d;
    ConfigurableMember              e;
    ConfigurableBase                f;
    ConfigurableDerived             g;
    ConfigurableMinimal             h;
    ConfigurableNoSetDefaults       k;
    ConfigurablePointers            l;
    ConfigurableAutoDeclare         m;
    ConfigurableNoAutoID            n;
    ConfigurableNoConstructors      o;
    ConfigurableVerbose             p;
    ConfigurableSpecialFloats       q;
    ConfigurableStrictness1         r;
    ConfigurableStrictness2         s;

    return (0);
}
