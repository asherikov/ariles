/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableAutoDeclare : public ariles2::DefaultBase
    {
// optional, but what is the point in omitting it?
// members can be defined manually, see ConfigurableVerbose
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
// mandatory
#include ARILES2_INITIALIZE


    public:
        ConfigurableAutoDeclare()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            integer_ = 10;
            real_ = 1.33;
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            integer_ = GET_RANDOM_INT;
            real_ = GET_RANDOM_REAL;
            ariles2::apply<ariles2::PostProcess>(*this);
        }
#endif
    };
}  // namespace ariles_tests
