/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableNoSetDefaults : public ariles::Base
    {
        #define ARILES_DEFAULT_ID "Configurable"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_TYPED_ENTRY_(real,        double)
        #include ARILES_INITIALIZE

        public:
            double another_real_;

        public:
            ConfigurableNoSetDefaults()
            {
                ariles<ariles::defaults::Visitor>();
                ariles<ariles::finalize::Visitor>();
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;
                integer_ = GET_RANDOM_INT;
                real_    = GET_RANDOM_REAL;
                ariles<ariles::finalize::Visitor>();
            }
#endif
    };
}
