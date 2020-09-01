/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


namespace ariles_tests
{
    class ConfigurableSpecialFloats : virtual public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, float_quiet_nan, float)                                                                    \
    ARILES2_TYPED_ENTRY_(v, float_signaling_nan, float)                                                                \
    ARILES2_TYPED_ENTRY_(v, float_positive_infinity, float)                                                            \
    ARILES2_TYPED_ENTRY_(v, float_negative_infinity, float)                                                            \
    ARILES2_TYPED_ENTRY_(v, double_quiet_nan, double)                                                                  \
    ARILES2_TYPED_ENTRY_(v, double_signaling_nan, double)                                                              \
    ARILES2_TYPED_ENTRY_(v, double_positive_infinity, double)                                                          \
    ARILES2_TYPED_ENTRY_(v, double_negative_infinity, double)
#include ARILES2_INITIALIZE

    public:
        bool postprocessed_;


    public:
        ConfigurableSpecialFloats()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        virtual ~ConfigurableSpecialFloats()
        {
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            postprocessed_ = false;

            float_quiet_nan_ = 0.0;
            float_signaling_nan_ = 0.0;
            float_positive_infinity_ = 0.0;
            float_negative_infinity_ = 0.0;

            double_quiet_nan_ = 0.0;
            double_signaling_nan_ = 0.0;
            double_positive_infinity_ = 0.0;
            double_negative_infinity_ = 0.0;
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        virtual void randomize()
        {
            boost::random::random_device random_generator;
            postprocessed_ = false;

            float_quiet_nan_ = std::numeric_limits<float>::quiet_NaN();
            float_signaling_nan_ = std::numeric_limits<float>::signaling_NaN();
            float_positive_infinity_ = std::numeric_limits<float>::infinity();
            float_negative_infinity_ = -std::numeric_limits<float>::infinity();

            double_quiet_nan_ = std::numeric_limits<double>::quiet_NaN();
            double_signaling_nan_ = std::numeric_limits<double>::signaling_NaN();
            double_positive_infinity_ = std::numeric_limits<double>::infinity();
            double_negative_infinity_ = -std::numeric_limits<double>::infinity();
        }
#endif

        void arilesVisit(const ariles2::PostProcess & /*visitor*/, const ariles2::PostProcess::Parameters & /*param*/)
        {
            postprocessed_ = true;
        }
    };



#ifndef ARILES_TESTS_COMPARE_DISABLED
    void check(class ConfigurableSpecialFloats &configurable)
    {
        BOOST_CHECK(true == configurable.postprocessed_);

        BOOST_CHECK(true == ariles2::isNaN(configurable.double_quiet_nan_));
        BOOST_CHECK(true == ariles2::isNaN(configurable.double_signaling_nan_));
        BOOST_CHECK(true == ariles2::isInfinity(configurable.double_positive_infinity_));
        BOOST_CHECK(true == ariles2::isInfinity(configurable.double_negative_infinity_));
        BOOST_CHECK(configurable.double_positive_infinity_ > 0.0);
        BOOST_CHECK(configurable.double_negative_infinity_ < 0.0);

        BOOST_CHECK(true == ariles2::isNaN(configurable.double_quiet_nan_));
        BOOST_CHECK(true == ariles2::isNaN(configurable.double_signaling_nan_));
        BOOST_CHECK(true == ariles2::isInfinity(configurable.double_positive_infinity_));
        BOOST_CHECK(true == ariles2::isInfinity(configurable.double_negative_infinity_));
        BOOST_CHECK(configurable.double_positive_infinity_ > 0.0);
        BOOST_CHECK(configurable.double_negative_infinity_ < 0.0);
    }
#endif
}  // namespace ariles_tests
