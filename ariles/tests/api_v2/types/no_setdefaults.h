/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableNoSetDefaults : public ariles::DefaultBase
    {
#define ARILES_ENTRIES                                                                                                 \
    ARILES_TYPED_ENTRY_(integer, int)                                                                                  \
    ARILES_TYPED_ENTRY_(real, double)
#include ARILES_INITIALIZE

    public:
        double another_real_;

    public:
        ConfigurableNoSetDefaults()
        {
            ariles::apply<ariles::Defaults>(*this);
            ariles::apply<ariles::PostProcess>(*this);
        }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            integer_ = GET_RANDOM_INT;
            real_ = GET_RANDOM_REAL;
            ariles::apply<ariles::PostProcess>(*this);
        }
#endif
    };
}  // namespace ariles_tests
