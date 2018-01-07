/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

// Enable YAML configuration files (must be first)
#include "ariles/format_yaml.h"
#include "ariles/format_msgpack.h"
// common & abstract classes
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


#include "fixture_000_basic_interface.h"
#include "fixture_002_comparison.h"
#include "fixture_003_comparison_vector.h"
#include "fixture_005_comparison_base.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(NAMESPACE) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableDerived) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, NAMESPACE, ConfigurableDerived) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, NAMESPACE, ConfigurableDerived) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, NAMESPACE, ConfigurableDerived) \
    BOOST_FIXTURE_TEST_CASE(ComparisonViaBaseFixture##_##NAMESPACE, ComparisonViaBaseFixture) \
    { \
        test<ConfigurableBase, ConfigurableDerived, ariles::NAMESPACE::Reader, ariles::NAMESPACE::Writer>(); \
    }

ARILES_TESTS(msgpack)
ARILES_TESTS(yaml)
