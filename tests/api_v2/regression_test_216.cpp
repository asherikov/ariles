/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#define ARILES_TESTS_RANDOMIZE_DISABLED
#define ARILES_TESTS_COMPARE_DISABLED

#include <api_v2/tests_config.h>
#include "all_enabled_adapters.h"

#include <ariles2/ariles.h>


// ===============================================================
// TYPES
// ===============================================================

#define ARILES_TEST_DEFAULT_BASE ariles2::DefaultBase

#include "types/any.h"
#include "types/complex_auto_declare.h"
#include "types/complex_base.h"
#include "types/complex_verbose.h"
#include "types/empty.h"
#include "types/enum.h"
#include "types/finalize.h"
#include "types/inheritance.h"
#include "types/minimal.h"
#include "types/auto_defaults.h"
#include "types/pointers.h"
#include "types/pointers_scalar.h"
#include "types/simple_auto_declare.h"
#include "types/simple_verbose.h"
#include "types/special_floats.h"
#include "types/strictness.h"

int main()
{
    ariles_tests::ConfigurableComplex a;
    ariles_tests::ConfigurableComplexVerbose b;
    ariles_tests::ConfigurableEmpty c;
    ariles_tests::ConfigurableFinalize d;
    ariles_tests::ConfigurableMember<int> e;
    ariles_tests::ConfigurableBase f;
    ariles_tests::ConfigurableDerived g;
    ariles_tests::ConfigurableMinimal h;
    ariles_tests::ConfigurableNoSetDefaults k;
    ariles_tests::ConfigurablePointers l;
    ariles_tests::ConfigurableAutoDeclare m;
    ariles_tests::ConfigurableVerbose p;
    ariles_tests::ConfigurableSpecialFloats q;
    ariles_tests::ConfigurableStrictness1 r;
    ariles_tests::ConfigurableStrictness2 s;
    ariles_tests::ConfigurableAny t;
    ariles_tests::ConfigurablePointersScalar u;

    return (0);
}
