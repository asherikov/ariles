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
    class ConfigurableSimpleFloats : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, float_member, float)                                                                       \
    ARILES2_TYPED_ENTRY_(v, double_member, double)
#include ARILES2_INITIALIZE


    public:
        ConfigurableSimpleFloats()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            float_member_ = 1;
            double_member_ = 1;
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            ariles2::apply<ariles2::Defaults>(*this);
            ariles2::apply<ariles2::PostProcess>(*this);
        }
#endif
    };


#ifndef ARILES_TESTS_COMPARE_DISABLED
    // comparison
    template <class t_Configurable_out, class t_Configurable_in>
    void compare(const t_Configurable_out &configurable_out, const t_Configurable_in &configurable_in)
    {
        BOOST_CHECK_CLOSE(configurable_out.double_member_, configurable_in.double_member_, g_tolerance);
        BOOST_CHECK_CLOSE(configurable_out.float_member_, configurable_in.float_member_, g_tolerance);
    }
#endif
}  // namespace ariles_tests
