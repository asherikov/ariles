/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"


#ifdef ARILES_BRIDGE_rapidjson
#include "ariles/bridges/rapidjson.h"
#endif

// If no format header is included, ariles is disabled, and
// ariles::ConfigurableBase is just a dummy class.
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================


/**
 * @brief Short definition of a configurable class -- types of members are
 * passed to Ariles for automatic declaration.
 */
class Configurable : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "Configurable"
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer,     int)
    #include ARILES_INITIALIZE


    public:
        Configurable()
        {
            setDefaults();
        }


        /**
         * @brief This method must be defined
         */
        virtual void setDefaults()
        {
            integer_ = 0;
        }
};


// ===============================================================
// FIXTURES
// ===============================================================

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
                string_id_ = "regression_test_010.json";
            }
    };

    typedef FilenameReaderInitializer<FilenameReaderBase> FilenameReaderInitializer010;
}


// ===============================================================
// TESTS
// ===============================================================


#define ARILES_TESTS(BRIDGE_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ReadFixture, BRIDGE_ID, NAMESPACE, Configurable, INITIALIZER)

ARILES_TESTS(rapidjson_jsonnet, rapidjson::jsonnet, FilenameReaderInitializer010)
