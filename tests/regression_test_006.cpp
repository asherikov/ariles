/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

// Enable ROS configuration files (must be first)
#include "ariles/formats/ros.h"

#include "ariles/adapters_all.h"

// definition of ariles::ConfigurableBase
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/strictness.h"
#include "types/complex.h"

class ConfigurableBase : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableBase"
    #define ARILES_CONSTRUCTOR ConfigurableBase
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(member,         ConfigurableComplex)
    #include ARILES_INITIALIZE


    public:
        ConfigurableBase()
        {
            setDefaults();
        }


        virtual void setDefaults()
        {
            member_.setDefaults();
        }


        void randomize()
        {
            member_.randomize();
            finalize();
        }
};


class ConfigurableDerived : public ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableDerived"
    #define ARILES_CONSTRUCTOR ConfigurableDerived
    #define ARILES_ENTRIES \
        ARILES_PARENT(ConfigurableBase) \
        ARILES_TYPED_ENTRY_(another_member,         ConfigurableComplex)
    #include ARILES_INITIALIZE


    public:
        ConfigurableDerived()
        {
            setDefaults();
        }


        virtual void setDefaults()
        {
            another_member_.setDefaults();
            ConfigurableBase::setDefaults();
        }


        void randomize()
        {
            another_member_.randomize();
            ConfigurableBase::randomize();
            finalize();
        }
};


// ===============================================================
// FIXTURES
// ===============================================================


#include "fixtures/base_ros.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/001_constructor_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"
#include "fixtures/005_comparison_base.h"
#include "fixtures/007_strictness.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(NAMESPACE) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableDerived) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, NAMESPACE, ConfigurableDerived) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, NAMESPACE, ConfigurableComplex) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, NAMESPACE, ConfigurableComplex) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, NAMESPACE, ConfigurableComplex) \
    ARILES_FIXTURE_TEST_CASE_2CLASSES(StrictnessFixture, NAMESPACE, ConfigurableStrictness1, ConfigurableStrictness2)

ARILES_TESTS(ros)
