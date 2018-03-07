/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

// Enable YAML configuration files (must be first)
#ifdef ARILES_BRIDGE_yaml_cpp03
#include "ariles/formats/yaml_cpp03.h"
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
#include "ariles/formats/yaml_cpp.h"
#endif

// all adapters
// #include "ariles/adapters_all.h"
// only Eigen adapters
#include "ariles/adapters/eigen.h"
// definition of ariles::ConfigurableBase
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
        ARILES_TYPED_ENTRY_(evector,     Eigen::Vector2d)
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
            evector_.setZero();
        }


        void randomize()
        {
            evector_.setRandom();
            finalize();
        }
};


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/base_default.h"
#include "fixtures/000_basic_interface.h"



// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(NAMESPACE) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, Configurable)

#ifdef ARILES_BRIDGE_yaml_cpp03
ARILES_TESTS(yaml_cpp03)
#endif
#ifdef ARILES_BRIDGE_yaml_cpp
ARILES_TESTS(yaml_cpp)
#endif
