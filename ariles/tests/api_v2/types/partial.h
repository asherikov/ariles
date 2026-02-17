/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/
// cppcheck-suppress-file duplInheritedMember

#pragma once


namespace ariles_tests
{
    class Partial : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE


    public:
        Partial()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            real_ = 1.33;
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            real_ = GET_RANDOM_REAL;
            ariles2::apply<ariles2::Finalize>(*this);
        }
#endif
    };


    class Complete : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE


    public:
        Complete()
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
            ariles2::apply<ariles2::Finalize>(*this);
        }
#endif
    };
}  // namespace ariles_tests
