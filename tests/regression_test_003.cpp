/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

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

#include "types/finalize.h"



// ===============================================================
// FIXTURES
// ===============================================================

// comparison
template<class t_Configurable_out, class t_Configurable_in>
void    compare(const t_Configurable_out    &configurable_out,
                const t_Configurable_in     &configurable_in)
{
    BOOST_CHECK_EQUAL(configurable_out.integer_,          configurable_in.integer_);
    BOOST_CHECK_CLOSE(configurable_out.real_,             configurable_in.real_, g_tolerance);
    BOOST_CHECK_CLOSE(configurable_out.another_real_,     configurable_in.another_real_, g_tolerance);
}


#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(BRIDGE_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, Configurable, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, BRIDGE_ID, NAMESPACE, Configurable, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, BRIDGE_ID, NAMESPACE, Configurable, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, BRIDGE_ID, NAMESPACE, Configurable, INITIALIZER)


#include "instantiate.h"
