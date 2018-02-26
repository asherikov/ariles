/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

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


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
        }
};


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/base_default.h"
#include "fixtures/006_dummy.h"



// ===============================================================
// TESTS
// ===============================================================


BOOST_FIXTURE_TEST_CASE( DummyFixture_Configurable, DummyFixture )
{
    test<Configurable>();
}
