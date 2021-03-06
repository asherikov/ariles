/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

namespace ariles_tests
{
    class ConfigurableFinalizeBase : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE

    public:
        double another_real_;
        bool defaults_check_flag_;

    public:
        ConfigurableFinalizeBase()
        {
            defaults_check_flag_ = false;

            ariles2::apply<ariles2::Defaults>(*this);
            ariles2::apply<ariles2::Finalize>(*this);
        }

        virtual ~ConfigurableFinalizeBase()
        {
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;

            defaults_check_flag_ = true;

            integer_ = 10;
            real_ = 1.33;
        }


        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            another_real_ = integer_ * real_;
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


    class ConfigurableFinalize : public ConfigurableFinalizeBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_PARENT(v, ConfigurableFinalizeBase)                                                                        \
    ARILES2_TYPED_ENTRY_(v, member, ConfigurableFinalizeBase)
#include ARILES2_INITIALIZE

    public:
        ConfigurableFinalize()
        {
            ariles2::apply<ariles2::Defaults>(*this);
            ariles2::apply<ariles2::Finalize>(*this);
        }

        virtual ~ConfigurableFinalize()
        {
        }



#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            ARILES2_TRACE_FUNCTION;
            boost::random::random_device random_generator;
            ConfigurableFinalizeBase::randomize();
            member_.randomize();
            ariles2::apply<ariles2::Finalize>(*this);
        }
#endif
    };


#ifndef ARILES_TESTS_COMPARE_DISABLED
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

        t_Configurable_in manual_finalize = configurable_in;
        manual_finalize.another_real_ = 0.0;
        manual_finalize.member_.another_real_ = 0.0;
        ariles2::apply<ariles2::Finalize>(manual_finalize);

        BOOST_CHECK_EQUAL(manual_finalize.integer_, configurable_in.integer_);
        BOOST_CHECK_CLOSE(manual_finalize.real_, configurable_in.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_finalize.another_real_, configurable_in.another_real_, g_tolerance);

        BOOST_CHECK_EQUAL(manual_finalize.member_.integer_, configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(manual_finalize.member_.real_, configurable_in.member_.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_finalize.member_.another_real_, configurable_in.member_.another_real_, g_tolerance);



        // Known issue of APIv1.
        manual_finalize.another_real_ = 0.0;
        manual_finalize.member_.another_real_ = 0.0;
        ariles2::apply<ariles2::Finalize>(manual_finalize);

        BOOST_CHECK_EQUAL(manual_finalize.integer_, configurable_in.integer_);
        BOOST_CHECK_CLOSE(manual_finalize.real_, configurable_in.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_finalize.another_real_, configurable_in.another_real_, g_tolerance);

        BOOST_CHECK_EQUAL(manual_finalize.member_.integer_, configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(manual_finalize.member_.real_, configurable_in.member_.real_, g_tolerance);
        BOOST_CHECK_EQUAL(manual_finalize.member_.another_real_, configurable_in.member_.another_real_);
    }
#endif
}  // namespace ariles_tests
