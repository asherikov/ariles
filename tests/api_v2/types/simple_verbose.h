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
    /**
     * @brief Verbose definition of a configurable class (with explicit declaration
     * of members)
     */
    class ConfigurableVerbose : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_ENTRY_(v, integer)                                                                                         \
    ARILES2_ENTRY_(v, real)
#include ARILES2_INITIALIZE


    public:
        int integer_;
        double real_;


    public:
        ConfigurableVerbose()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
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
            ariles2::apply<ariles2::PostProcess>(*this);
        }
#endif
    };
}  // namespace ariles_tests
