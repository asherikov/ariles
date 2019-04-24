/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


namespace ariles_tests
{
    class ConfigurableSpecialFloats : virtual public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "ConfigurableSpecialFloats"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(float_quiet_nan, float) \
            ARILES_TYPED_ENTRY_(float_signaling_nan, float) \
            ARILES_TYPED_ENTRY_(float_positive_infinity, float) \
            ARILES_TYPED_ENTRY_(float_negative_infinity, float) \
            ARILES_TYPED_ENTRY_(double_quiet_nan, double) \
            ARILES_TYPED_ENTRY_(double_signaling_nan, double) \
            ARILES_TYPED_ENTRY_(double_positive_infinity, double) \
            ARILES_TYPED_ENTRY_(double_negative_infinity, double)
        #include ARILES_INITIALIZE

        public:
            bool finalized_;


        public:
            ConfigurableSpecialFloats()
            {
                setDefaults();
            }

            virtual ~ConfigurableSpecialFloats() {}


            virtual void setDefaults()
            {
                finalized_ = false;


                float_quiet_nan_ = 0.0;
                float_signaling_nan_ = 0.0;
                float_positive_infinity_ = 0.0;
                float_negative_infinity_ = 0.0;

                double_quiet_nan_ = 0.0;
                double_signaling_nan_ = 0.0;
                double_positive_infinity_ = 0.0;
                double_negative_infinity_ = 0.0;
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            virtual void randomize()
            {
                finalized_ = false;

                float_quiet_nan_ = std::numeric_limits<float>::quiet_NaN();
                float_signaling_nan_ = std::numeric_limits<float>::signaling_NaN();
                float_positive_infinity_ = std::numeric_limits<float>::infinity();
                float_negative_infinity_ = - std::numeric_limits<float>::infinity();

                double_quiet_nan_ = std::numeric_limits<double>::quiet_NaN();
                double_signaling_nan_ = std::numeric_limits<double>::signaling_NaN();
                double_positive_infinity_ = std::numeric_limits<double>::infinity();
                double_negative_infinity_ = - std::numeric_limits<double>::infinity();
            }
#endif

            virtual void finalize()
            {
                finalized_ = true;
            }
    };



#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
    void check(class ConfigurableSpecialFloats & configurable)
    {
        BOOST_CHECK(true == configurable.finalized_);

        BOOST_CHECK(true == ariles::isNaN(configurable.double_quiet_nan_));
        BOOST_CHECK(true == ariles::isNaN(configurable.double_signaling_nan_));
        BOOST_CHECK(true == ariles::isInfinity(configurable.double_positive_infinity_));
        BOOST_CHECK(true == ariles::isInfinity(configurable.double_negative_infinity_));
        BOOST_CHECK(configurable.double_positive_infinity_ > 0.0);
        BOOST_CHECK(configurable.double_negative_infinity_ < 0.0);

        BOOST_CHECK(true == ariles::isNaN(configurable.double_quiet_nan_));
        BOOST_CHECK(true == ariles::isNaN(configurable.double_signaling_nan_));
        BOOST_CHECK(true == ariles::isInfinity(configurable.double_positive_infinity_));
        BOOST_CHECK(true == ariles::isInfinity(configurable.double_negative_infinity_));
        BOOST_CHECK(configurable.double_positive_infinity_ > 0.0);
        BOOST_CHECK(configurable.double_negative_infinity_ < 0.0);
    }
#endif
}
