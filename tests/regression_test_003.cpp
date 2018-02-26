/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "utility.h"

#include "ariles/ariles_all.h"


// ===============================================================
// TYPES
// ===============================================================

class Configurable : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "Configurable"
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer,     int) \
        ARILES_TYPED_ENTRY_(real,        double)
    #include ARILES_INITIALIZE

    public:
        double another_real_;

    public:
        Configurable()
        {
            setDefaults();
            finalize();
        }


        virtual void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
        }


        virtual void finalize()
        {
            another_real_ = integer_ * real_;
        }


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
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
    BOOST_CHECK_EQUAL(configurable_out.integer_,          configurable_in.integer_);
    BOOST_CHECK_CLOSE(configurable_out.real_,             configurable_in.real_, g_tolerance);
    BOOST_CHECK_CLOSE(configurable_out.another_real_,     configurable_in.another_real_, g_tolerance);
}


#include "fixtures/base_default.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/002_comparison.h"
#include "fixtures/003_comparison_vector.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(NAMESPACE) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, Configurable) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, NAMESPACE, Configurable) \
    ARILES_FIXTURE_TEST_CASE(ComparisonMultiFixture, NAMESPACE, Configurable) \
    ARILES_FIXTURE_TEST_CASE(ComparisonVectorFixture, NAMESPACE, Configurable)

ARILES_TESTS(msgpack)
ARILES_TESTS(yaml)
