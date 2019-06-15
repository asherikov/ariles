/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

namespace ariles_tests
{
    class ConfigurableFinalizeBase : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "ConfigurableFinalizeBase"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_TYPED_ENTRY_(real,        double)
        #include ARILES_INITIALIZE

        public:
            double another_real_;

        public:
            ConfigurableFinalizeBase()
            {
                setDefaults();
                finalize();
            }

            virtual ~ConfigurableFinalizeBase() {}


            virtual void setDefaults()
            {
                integer_ = 10;
                real_ = 1.33;
            }


            virtual void finalize()
            {
                another_real_ = integer_ * real_;
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;
                integer_ = GET_RANDOM_INT;
                real_    = GET_RANDOM_REAL;
                finalize();
            }
#endif
    };


    class ConfigurableFinalize : public ConfigurableFinalizeBase
    {
        #define ARILES_SECTION_ID "ConfigurableFinalize"
        #define ARILES_ENTRIES \
            ARILES_PARENT(ConfigurableFinalizeBase) \
            ARILES_TYPED_ENTRY_(member, ConfigurableFinalizeBase)
        #define ARILES_AUTO_DEFAULTS
        #include ARILES_INITIALIZE

        public:
            ConfigurableFinalize()
            {
                setDefaults();
                arilesFinalize();
            }

            virtual ~ConfigurableFinalize() {}




#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;
                ConfigurableFinalizeBase::randomize();
                member_.randomize();
                arilesFinalize();
            }
#endif
    };


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
    // comparison
    template<class t_Configurable_out, class t_Configurable_in>
    void    compare(const t_Configurable_out    &configurable_out,
                    const t_Configurable_in     &configurable_in)
    {
        BOOST_CHECK_EQUAL(configurable_out.integer_,          configurable_in.integer_);
        BOOST_CHECK_CLOSE(configurable_out.real_,             configurable_in.real_, g_tolerance);
        BOOST_CHECK_CLOSE(configurable_out.another_real_,     configurable_in.another_real_, g_tolerance);

        BOOST_CHECK_EQUAL(configurable_out.member_.integer_,          configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(configurable_out.member_.real_,             configurable_in.member_.real_, g_tolerance);
        BOOST_CHECK_CLOSE(configurable_out.member_.another_real_,     configurable_in.member_.another_real_, g_tolerance);

        t_Configurable_in manual_finalize = configurable_in;
        manual_finalize.another_real_ = 0.0;
        manual_finalize.member_.another_real_ = 0.0;
        manual_finalize.arilesFinalize();

        BOOST_CHECK_EQUAL(manual_finalize.integer_,          configurable_in.integer_);
        BOOST_CHECK_CLOSE(manual_finalize.real_,             configurable_in.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_finalize.another_real_,     configurable_in.another_real_, g_tolerance);

        BOOST_CHECK_EQUAL(manual_finalize.member_.integer_,          configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(manual_finalize.member_.real_,             configurable_in.member_.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_finalize.member_.another_real_,     configurable_in.member_.another_real_, g_tolerance);


        // Known issue.
        manual_finalize.another_real_ = 0.0;
        manual_finalize.member_.another_real_ = 0.0;
        manual_finalize.finalize();

        BOOST_CHECK_EQUAL(manual_finalize.integer_,          configurable_in.integer_);
        BOOST_CHECK_CLOSE(manual_finalize.real_,             configurable_in.real_, g_tolerance);
        BOOST_CHECK_CLOSE(manual_finalize.another_real_,     configurable_in.another_real_, g_tolerance);

        BOOST_CHECK_EQUAL(manual_finalize.member_.integer_,          configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(manual_finalize.member_.real_,             configurable_in.member_.real_, g_tolerance);
        BOOST_CHECK_EQUAL(manual_finalize.member_.another_real_,     0.0); // <--
    }
#endif
}
