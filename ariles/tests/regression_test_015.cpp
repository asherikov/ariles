/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"


#include "ariles/bridges/pugixml.h"

// If no format header is included, ariles is disabled, and
// ariles::ConfigurableBase is just a dummy class.
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/simple_auto_declare.h"


// ===============================================================
// FIXTURES
// ===============================================================

#undef ARILES_BRIDGE_ros
#include "fixtures/initializers.h"
#include "fixtures/009_read.h"

namespace initializers
{
    class FilenameReaderBase
    {
        public:
            std::string string_id_;

        public:
            FilenameReaderBase()
            {
                string_id_ = "regression_test_015.xml";
            }
    };

    typedef FilenameReaderInitializer<FilenameReaderBase> FilenameReaderInitializer015;
}


// ===============================================================
// TESTS
// ===============================================================


#define ARILES_TESTS(BRIDGE_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ReadFixture, BRIDGE_ID, NAMESPACE, ConfigurableAutoDeclare, INITIALIZER)

ARILES_TESTS(pugixml, pugixml, FilenameReaderInitializer015)
