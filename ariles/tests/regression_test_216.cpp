/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#define ARILES_API_VERSION 2


#define ARILES_TESTS_BOOST_UTF_DISABLED

#include <tests_config.h>

#include "ariles/adapters_all.h"
#include "ariles/ariles2.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types_api_v2/any.h"
#include "types_api_v2/complex_auto_declare.h"
#include "types_api_v2/complex_base.h"
#include "types_api_v2/complex_verbose.h"
#include "types_api_v2/empty.h"
#include "types_api_v2/enum.h"
#include "types_api_v2/postprocess.h"
#include "types_api_v2/inheritance.h"
#include "types_api_v2/minimal.h"
#include "types_api_v2/no_setdefaults.h"
#include "types_api_v2/pointers.h"
#include "types_api_v2/pointers_scalar.h"
#include "types_api_v2/simple_auto_declare.h"
#include "types_api_v2/simple_no_auto_id.h"
#include "types_api_v2/simple_verbose.h"
#include "types_api_v2/special_floats.h"
#include "types_api_v2/strictness.h"

int main()
{
    ariles_tests::ConfigurableComplex             a;
    ariles_tests::ConfigurableComplexVerbose      b;
    ariles_tests::ConfigurableEmpty               c;
    ariles_tests::ConfigurablePostProcess         d;
    ariles_tests::ConfigurableMember<int>         e;
    ariles_tests::ConfigurableBase                f;
    ariles_tests::ConfigurableDerived             g;
    ariles_tests::ConfigurableMinimal             h;
    ariles_tests::ConfigurableNoSetDefaults       k;
    ariles_tests::ConfigurablePointers            l;
    ariles_tests::ConfigurableAutoDeclare         m;
    ariles_tests::ConfigurableNoAutoID            n;
    ariles_tests::ConfigurableVerbose             p;
    ariles_tests::ConfigurableSpecialFloats       q;
    ariles_tests::ConfigurableStrictness1         r;
    ariles_tests::ConfigurableStrictness2         s;
    ariles_tests::ConfigurableAny                 t;
    ariles_tests::ConfigurablePointersScalar      u;

    return (0);
}
