/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableNoSetDefaults : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "Configurable"
        #define ARILES_AUTO_DEFAULTS
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_TYPED_ENTRY_(real,        double)
        #include ARILES_INITIALIZE

        public:
            double another_real_;

        public:
            ConfigurableNoSetDefaults()
            {
                setDefaults();
                finalize();
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                integer_ = GET_RANDOM_INT;
                real_    = GET_RANDOM_REAL;
                finalize();
            }
#endif
    };
}
