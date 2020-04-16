/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#define ARILES_API_VERSION 2

#include "utility.h"


#include "ariles/visitors/jsonnet.h"
#include "ariles/visitors/rapidjson.h"

// If no format header is included, ariles is disabled, and
// ariles::ConfigurableBase is just a dummy class.
#include "ariles/ariles2.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types_api_v2/simple_auto_declare.h"


// ===============================================================
// FIXTURES
// ===============================================================

#undef ARILES_BRIDGE_ros
#include "fixtures_api_v2/initializers.h"
#include "fixtures_api_v2/009_read.h"

namespace ariles_tests
{
    namespace initializers
    {
        class FilenameReaderBase
        {
            public:
                std::string string_id_;

            public:
                FilenameReaderBase()
                {
                    string_id_ = "regression_test_210.json";
                }
        };

        typedef FilenameReaderInitializer<FilenameReaderBase> FilenameReaderInitializer210;
    }
}

// ===============================================================
// TESTS
// ===============================================================


#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ReadFixture, VISITOR_ID, NAMESPACE, ConfigurableAutoDeclare, INITIALIZER)

ARILES_TESTS(rapidjson_jsonnet, jsonnet<ariles::rapidjson>, FilenameReaderInitializer210)
