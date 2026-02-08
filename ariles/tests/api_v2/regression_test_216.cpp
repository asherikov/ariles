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
    const ariles_tests::ConfigurableComplex a;
    const ariles_tests::ConfigurableComplexVerbose b;
    const ariles_tests::ConfigurableEmpty c;
    const ariles_tests::ConfigurableFinalize d;
    const ariles_tests::ConfigurableMember<int> e;
    const ariles_tests::ConfigurableBase f;
    const ariles_tests::ConfigurableDerived g;
    const ariles_tests::ConfigurableMinimal h{};
    const ariles_tests::ConfigurableNoSetDefaults k;
    const ariles_tests::ConfigurablePointers l;
    const ariles_tests::ConfigurableAutoDeclare m;
    const ariles_tests::ConfigurableVerbose p;
    const ariles_tests::ConfigurableSpecialFloats q;
    const ariles_tests::ConfigurableStrictness1 r;
    const ariles_tests::ConfigurableStrictness2 s;
    const ariles_tests::ConfigurableAny t;
    const ariles_tests::ConfigurablePointersScalar u;

    return (0);
}
