/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"


#ifdef ARILES_VISITOR_protobuf3
#    define ARILES2_DEFAULT_VISITORS                                                                                   \
        ARILES2_VISITOR(count)                                                                                         \
        ARILES2_VISITOR(finalize)                                                                                      \
        ARILES2_VISITOR(prewrite)                                                                                      \
        ARILES2_VISITOR(defaults)                                                                                      \
        ARILES2_VISITOR(read)                                                                                          \
        ARILES2_VISITOR(write)                                                                                         \
        ARILES2_VISITOR(compare)                                                                                       \
        ARILES2_VISITOR(protobuf3)

#    include <ariles2/visitors/compare.h>
#    include <ariles2/visitors/protobuf3.h>
#endif

#include <ariles2/ariles.h>
#include <ariles2/extra.h>

#include "all_enabled_adapters.h"



// ===============================================================
// TYPES
// ===============================================================

#define ARILES_TESTS_COMPARE_DISABLED
#include "test.pb.h"
#include "types/protobuf/test.h"
#undef ARILES_TESTS_COMPARE_DISABLED


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/018_copy_compare.h"

#define ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(ID, CONFIGURABLE_TYPE1, CONFIGURABLE_TYPE2)                              \
    BOOST_FIXTURE_TEST_CASE(CopyCompareFixture_##ID, ariles_tests::CopyCompareFixture)                                 \
    {                                                                                                                  \
        test<ariles2::protobuf3::Writer,                                                                               \
             ariles2::protobuf3::Reader,                                                                               \
             ariles_tests::CONFIGURABLE_TYPE1,                                                                         \
             CONFIGURABLE_TYPE2>();                                                                                    \
    }


// ===============================================================
// TESTS
// ===============================================================

ARILES_FIXTURE_TEST_CASE_COPY_COMPARE(Scalars, protobuf::Scalars, Scalars)
