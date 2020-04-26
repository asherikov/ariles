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
    class ConfigurableAutoDeclare : public ariles::DefaultBase
    {
// optional, but what is the point in omitting it?
// members can be defined manually, see ConfigurableVerbose
#define ARILES_ENTRIES                                                                                                 \
    ARILES_TYPED_ENTRY_(integer, int)                                                                                  \
    ARILES_TYPED_ENTRY_(real, double)
// mandatory
#include ARILES_INITIALIZE


    public:
        ConfigurableAutoDeclare()
        {
            ariles::apply<ariles::Defaults>(*this);
        }


        void arilesVisit(const ariles::Defaults & /*visitor*/, const ariles::Defaults::Parameters & /*param*/)
        {
            integer_ = 10;
            real_ = 1.33;
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
