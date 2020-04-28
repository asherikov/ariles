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
    class ConfigurableSimpleFloats : public ariles::DefaultBase
    {
#define ARILES_ENTRIES                                                                                                 \
    ARILES_TYPED_ENTRY_(float_member, float)                                                                           \
    ARILES_TYPED_ENTRY_(double_member, double)
#include ARILES_INITIALIZE


    public:
        ConfigurableSimpleFloats()
        {
            ariles::apply<ariles::Defaults>(*this);
        }


        void arilesVisit(const ariles::Defaults & /*visitor*/, const ariles::Defaults::Parameters & /*param*/)
        {
            float_member_ = 1;
            double_member_ = 1;
        }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
        void randomize()
        {
            ariles::apply<ariles::Defaults>(*this);
            ariles::apply<ariles::PostProcess>(*this);
        }
#endif
    };


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
    // comparison
    template <class t_Configurable_out, class t_Configurable_in>
    void compare(const t_Configurable_out &configurable_out, const t_Configurable_in &configurable_in)
    {
        BOOST_CHECK_CLOSE(configurable_out.double_member_, configurable_in.double_member_, g_tolerance);
        BOOST_CHECK_CLOSE(configurable_out.float_member_, configurable_in.float_member_, g_tolerance);
    }
#endif
}  // namespace ariles_tests
