/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

namespace ariles_tests
{
    class ConfigurablePostProcessBase : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE

    public:
        double another_real_;

    public:
        ConfigurablePostProcessBase()
        {
            ariles2::apply<ariles2::Defaults>(*this);
            ariles2::apply<ariles2::PostProcess>(*this);
        }

        virtual ~ConfigurablePostProcessBase()
        {
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            integer_ = 10;
            real_ = 1.33;
        }


        void arilesVisit(const ariles2::PostProcess & /*visitor*/, const ariles2::PostProcess::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            another_real_ = integer_ * real_;
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


    class ConfigurablePostProcess : public ConfigurablePostProcessBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_PARENT(v, ConfigurablePostProcessBase)                                                                     \
    ARILES2_TYPED_ENTRY_(v, member, ConfigurablePostProcessBase)
#include ARILES2_INITIALIZE

    public:
        ConfigurablePostProcess()
        {
            ariles2::apply<ariles2::Defaults>(*this);
            ariles2::apply<ariles2::PostProcess>(*this);
        }

        virtual ~ConfigurablePostProcess()
        {
        }



#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
        void randomize()
        {
            ARILES2_TRACE_FUNCTION;
            boost::random::random_device random_generator;
            ConfigurablePostProcessBase::randomize();
            member_.randomize();
            ariles2::apply<ariles2::PostProcess>(*this);
        }
#endif
    };


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
    // comparison
    template <class t_Configurable_out, class t_Configurable_in>
    void compare(const t_Configurable_out &configurable_out, const t_Configurable_in &configurable_in)
    {
        BOOST_CHECK_EQUAL(configurable_out.integer_, configurable_in.integer_);
        BOOST_CHECK_CLOSE(configurable_out.real_, configurable_in.real_, g_tolerance);
        BOOST_CHECK_CLOSE(configurable_out.another_real_, configurable_in.another_real_, g_tolerance);

        BOOST_CHECK_EQUAL(configurable_out.member_.integer_, configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(configurable_out.member_.real_, configurable_in.member_.real_, g_tolerance);
        BOOST_CHECK_CLOSE(configurable_out.member_.another_real_, configurable_in.member_.another_real_, g_tolerance);

        t_Configurable_in manual_postprocess = configurable_in;
        manual_postprocess.another_real_ = 0.0;
        manual_postprocess.member_.another_real_ = 0.0;
        ariles2::apply<ariles2::PostProcess>(manual_postprocess);

        BOOST_CHECK_EQUAL(manual_postprocess.integer_, configurable_in.integer_);
        BOOST_CHECK_CLOSE(manual_postprocess.real_, configurable_in.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_postprocess.another_real_, configurable_in.another_real_, g_tolerance);

        BOOST_CHECK_EQUAL(manual_postprocess.member_.integer_, configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(manual_postprocess.member_.real_, configurable_in.member_.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_postprocess.member_.another_real_, configurable_in.member_.another_real_, g_tolerance);



        // Known issue of APIv1.
        manual_postprocess.another_real_ = 0.0;
        manual_postprocess.member_.another_real_ = 0.0;
        ariles2::apply<ariles2::PostProcess>(manual_postprocess);

        BOOST_CHECK_EQUAL(manual_postprocess.integer_, configurable_in.integer_);
        BOOST_CHECK_CLOSE(manual_postprocess.real_, configurable_in.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_postprocess.another_real_, configurable_in.another_real_, g_tolerance);

        BOOST_CHECK_EQUAL(manual_postprocess.member_.integer_, configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(manual_postprocess.member_.real_, configurable_in.member_.real_, g_tolerance);
        BOOST_CHECK_EQUAL(manual_postprocess.member_.another_real_, configurable_in.member_.another_real_);
    }
#endif
}  // namespace ariles_tests