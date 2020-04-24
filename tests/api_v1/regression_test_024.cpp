/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#define ARILES_DEFAULT_CONFIGURABLE_FLAGS                                                          \
    ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED                                            \
            | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED

#ifdef ARILES_BRIDGE_yaml_cpp03
#    include "ariles/bridges/yaml_cpp03.h"
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
#    include "ariles/bridges/yaml_cpp.h"
#endif

#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

namespace ariles_tests
{
    struct SubstateParams : public ariles::RelaxedConfigurableBase
    {
#define ARILES_SECTION_ID "SubstateParams"
#define ARILES_ENTRIES                                                                             \
    ARILES_ENTRY(type)                                                                             \
    ARILES_ENTRY(remappings)
#include ARILES_INITIALIZE

        std::string type;
        std::map<std::string, std::string> remappings;

        virtual ~SubstateParams()
        {
        }
    };

    struct StateMachineParams : public ariles::RelaxedConfigurableBase
    {
#define ARILES_SECTION_ID "StateMachineParams"
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

#undef ARILES_BRIDGE_ros
#include "fixtures/initializers.h"
#include "fixtures/009_read.h"

namespace ariles_tests
{
    namespace initializers
    {
        class FilenameReader024
        {
        public:
            std::string string_id_;

        public:
            FilenameReader024()
            {
                string_id_ = "regression_test_024.yaml";
            }
        };

        typedef FilenameReaderInitializer<FilenameReader024> FilenameReaderInitializer024;
    }  // namespace initializers
}  // namespace ariles_tests


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(VISITOR_ID, NAMESPACE, INITIALIZER)                                           \
    ARILES_FIXTURE_TEST_CASE(ReadFixture, VISITOR_ID, NAMESPACE, StateMachineParams, INITIALIZER)

#ifdef ARILES_VISITOR_INCLUDED_yaml_cpp
ARILES_TESTS(yaml_cpp, yaml_cpp, FilenameReaderInitializer024)
#endif

#ifdef ARILES_VISITOR_INCLUDED_yaml_cpp03
ARILES_TESTS(yaml_cpp03, yaml_cpp03, FilenameReaderInitializer024)
#endif
