/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#define ARILES_DEFAULT_CONFIGURABLE_FLAGS                                                                              \
    ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED         \
            | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES

#ifdef ARILES_VISITOR_yaml_cpp03
#    include "ariles/visitors/yaml_cpp03.h"
#endif

#ifdef ARILES_VISITOR_yaml_cpp
#    include "ariles/visitors/yaml_cpp.h"
#endif

#include "all_enabled_adapters.h"

#include "ariles/ariles2.h"


// ===============================================================
// TYPES
// ===============================================================

namespace ariles_tests
{
    struct SubstateParams : public ariles::DefaultBase
    {
#define ARILES_ENTRIES                                                                                                 \
    ARILES_ENTRY(type)                                                                                                 \
    ARILES_ENTRY(remappings)
#include ARILES_INITIALIZE

        std::string type;
        std::map<std::string, std::string> remappings;

        virtual ~SubstateParams()
        {
        }
    };

    struct StateMachineParams : public ariles::DefaultBase
    {
#define ARILES_ENTRIES ARILES_ENTRY(substates)
#include ARILES_INITIALIZE

        std::map<std::string, SubstateParams> substates;

        virtual ~StateMachineParams()
        {
        }
    };
}  // namespace ariles_tests



// ===============================================================
// FIXTURES
// ===============================================================

#undef ARILES_VISITOR_ros
#include "fixtures/initializers.h"
#include "fixtures/009_read.h"

namespace ariles_tests
{
    namespace initializers
    {
        class FilenameReader224
        {
        public:
            std::string string_id_;

        public:
            FilenameReader224()
            {
                string_id_ = "regression_test_224.yaml";
            }
        };

        typedef FilenameReaderInitializer<FilenameReader224> FilenameReaderInitializer224;
    }  // namespace initializers
}  // namespace ariles_tests


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                                               \
    ARILES_FIXTURE_TEST_CASE(ReadFixture, VISITOR_ID, NAMESPACE, StateMachineParams, INITIALIZER)

#ifdef ARILES_VISITOR_INCLUDED_yaml_cpp03
ARILES_TESTS(yaml_cpp03, yaml_cpp03, FilenameReaderInitializer224)
#endif

#ifdef ARILES_VISITOR_INCLUDED_yaml_cpp
ARILES_TESTS(yaml_cpp, yaml_cpp, FilenameReaderInitializer224)
#endif
