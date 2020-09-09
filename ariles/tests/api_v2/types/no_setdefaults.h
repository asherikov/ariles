/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableNoSetDefaults : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE

    public:
        double another_real_;

    public:
        ConfigurableNoSetDefaults()
        {
            ariles2::apply<ariles2::Defaults>(*this);
            ariles2::apply<ariles2::PostRead>(*this);
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            integer_ = GET_RANDOM_INT;
            real_ = GET_RANDOM_REAL;
            ariles2::apply<ariles2::PostRead>(*this);
        }
#endif
    };
}  // namespace ariles_tests
