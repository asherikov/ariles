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

#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================


class ConfigurableMember : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableMember"
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer, int) \
        ARILES_TYPED_ENTRY_(real   , double)
    #include ARILES_INITIALIZE


    public:
        ConfigurableMember()
        {
            setDefaults();
        }

        virtual ~ConfigurableMember() {}


        virtual void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
        }


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
            finalize();
        }
};


class ConfigurableBase : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableBase"
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(member,         ConfigurableMember)
    #include ARILES_INITIALIZE


    public:
        ConfigurableBase()
        {
            setDefaults();
        }

        virtual ~ConfigurableBase() {}


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
    #define ARILES_ENTRIES \
        ARILES_PARENT(ConfigurableBase) \
        ARILES_TYPED_ENTRY_(another_member,         ConfigurableMember)
    #include ARILES_INITIALIZE


    public:
        ConfigurableDerived()
        {
            setDefaults();
        }


        void setDefaults()
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

#include "types/strictness.h"



// ===============================================================
// FIXTURES
// ===============================================================

// comparison
template<class t_Configurable_out, class t_Configurable_in>
void    compare(const t_Configurable_out    &configurable_out,
                const t_Configurable_in     &configurable_in)
{
    BOOST_CHECK_EQUAL(configurable_out.another_member_.integer_, configurable_in.another_member_.integer_);
    BOOST_CHECK_CLOSE(configurable_out.another_member_.real_,    configurable_in.another_member_.real_, g_tolerance);
    BOOST_CHECK_EQUAL(configurable_out.member_.integer_,         configurable_in.member_.integer_);
    BOOST_CHECK_CLOSE(configurable_out.member_.real_,            configurable_in.member_.real_, g_tolerance);
}


#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"
#include "fixtures/005_comparison_base.h"
#include "fixtures/007_strictness.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(BRIDGE_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableDerived, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, BRIDGE_ID, NAMESPACE, ConfigurableDerived, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, BRIDGE_ID, NAMESPACE, ConfigurableDerived, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, BRIDGE_ID, NAMESPACE, ConfigurableDerived, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE_2CLASSES(StrictnessFixture, BRIDGE_ID, NAMESPACE, ConfigurableStrictness1, ConfigurableStrictness2, INITIALIZER) \
    BOOST_FIXTURE_TEST_CASE(ComparisonViaBaseFixture##_##BRIDGE_ID##_##INITIALIZER, ComparisonViaBaseFixture<initializers::INITIALIZER>) \
    { \
        test<ConfigurableBase, ConfigurableDerived, ariles::NAMESPACE>(); \
    }


#include "instantiate.h"
