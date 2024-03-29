/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

#include <iostream>
namespace ariles_tests
{
    class ConfigurablePointersScalar : public ariles2::DefaultBase
    {
#define ARILES2_DEFAULT_ID "ConfigurablePointersScalar"
#define ARILES2_ENTRIES_0(v)

#define ARILES2_ENTRIES_1(v)                                                                                           \
    ARILES2_ENTRIES_0(v)                                                                                               \
    ARILES2_TYPED_ENTRY_(v, std_shared_ptr_real, std::shared_ptr<double>)                                              \
    ARILES2_TYPED_ENTRY_(v, std_unique_ptr_real, std::unique_ptr<double>)


#ifdef ARILES_ADAPTER_BOOST_POINTER
#    if BOOST_VERSION >= 105800
#        define ARILES2_ENTRIES_2(v)                                                                                   \
            ARILES2_ENTRIES_1(v)                                                                                       \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_real, boost::shared_ptr<double>)                                        \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_real_null, boost::shared_ptr<double>)                                   \
            ARILES2_TYPED_ENTRY_(v, unique_ptr_real, boost::movelib::unique_ptr<double>)
#    else
#        define ARILES2_ENTRIES_2(v)                                                                                   \
            ARILES2_ENTRIES_1(v)                                                                                       \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_real, boost::shared_ptr<double>)                                        \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_real_null, boost::shared_ptr<double>)
#    endif
#else
#    define ARILES2_ENTRIES_2(v) ARILES2_ENTRIES_1(v)
#endif


#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
#    define ARILES2_ENTRIES_3(v)                                                                                       \
        ARILES2_ENTRIES_2(v)                                                                                           \
        ARILES2_TYPED_ENTRY_(v, optional_real, boost::optional<double>)                                                \
        ARILES2_TYPED_ENTRY_(v, optional_real_null, boost::optional<double>)
#else
#    define ARILES2_ENTRIES_3(v) ARILES2_ENTRIES_2(v)
#endif


#define ARILES2_ENTRIES(v) ARILES2_ENTRIES_3(v)
#include ARILES2_INITIALIZE

#undef ARILES2_ENTRIES_0
#undef ARILES2_ENTRIES_1
#undef ARILES2_ENTRIES_2
#undef ARILES2_ENTRIES_3


    public:
        ConfigurablePointersScalar()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            std_shared_ptr_real_.reset();
            std_unique_ptr_real_.reset();

#ifdef ARILES_ADAPTER_BOOST_POINTER
            shared_ptr_real_.reset();
            shared_ptr_real_null_.reset();
#    if BOOST_VERSION >= 105800
            unique_ptr_real_.reset();
#    endif
#endif

#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
            optional_real_ = boost::none;
            optional_real_null_ = boost::none;
#endif
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            std_shared_ptr_real_ = std::make_shared<double>();
            *std_shared_ptr_real_ = GET_RANDOM_REAL;

            std_unique_ptr_real_.reset(new double());
            *std_unique_ptr_real_ = GET_RANDOM_REAL;

#    ifdef ARILES_ADAPTER_BOOST_POINTER
            shared_ptr_real_ = boost::make_shared<double>();
            *shared_ptr_real_ = GET_RANDOM_REAL;

#        if BOOST_VERSION >= 105800
            unique_ptr_real_ = boost::movelib::make_unique<double>();
            *unique_ptr_real_ = GET_RANDOM_REAL;
#        endif

            shared_ptr_real_null_.reset();
#    endif

#    ifdef ARILES_ADAPTER_BOOST_OPTIONAL
            optional_real_ = GET_RANDOM_REAL;
            optional_real_null_ = boost::none;
#    endif
        }
#endif
    };


#ifndef ARILES_TESTS_COMPARE_DISABLED
    template <class t_Configurable_out, class t_Configurable_in>
    void compare(const t_Configurable_out &configurable_out, const t_Configurable_in &configurable_in)
    {
        if (configurable_in.std_shared_ptr_real_ == NULL)
        {
            BOOST_CHECK_EQUAL(configurable_out.std_shared_ptr_real_, configurable_in.std_shared_ptr_real_);
        }
        else
        {
            BOOST_CHECK(configurable_out.std_shared_ptr_real_ != NULL);
            BOOST_CHECK_CLOSE(
                    *configurable_out.std_shared_ptr_real_, *configurable_in.std_shared_ptr_real_, g_tolerance);
        }
        if (configurable_in.std_unique_ptr_real_ == NULL)
        {
            BOOST_CHECK(configurable_out.std_unique_ptr_real_ == configurable_in.std_unique_ptr_real_);
        }
        else
        {
            BOOST_CHECK(configurable_out.std_unique_ptr_real_ != NULL);
            BOOST_CHECK_CLOSE(
                    *configurable_out.std_unique_ptr_real_, *configurable_in.std_unique_ptr_real_, g_tolerance);
        }


#    ifdef ARILES_ADAPTER_BOOST_POINTER
        if (configurable_in.shared_ptr_real_ == NULL)
        {
            BOOST_CHECK_EQUAL(configurable_out.shared_ptr_real_, configurable_in.shared_ptr_real_);
        }
        else
        {
            BOOST_CHECK(configurable_out.shared_ptr_real_ != NULL);
            BOOST_CHECK_CLOSE(*configurable_out.shared_ptr_real_, *configurable_in.shared_ptr_real_, g_tolerance);
        }
#        if BOOST_VERSION >= 105800
        if (configurable_in.unique_ptr_real_ == NULL)
        {
            BOOST_CHECK(configurable_out.unique_ptr_real_ == configurable_in.unique_ptr_real_);
        }
        else
        {
            BOOST_CHECK(configurable_out.unique_ptr_real_ != NULL);
            BOOST_CHECK_CLOSE(*configurable_out.unique_ptr_real_, *configurable_in.unique_ptr_real_, g_tolerance);
        }
#        endif

        BOOST_CHECK(configurable_out.shared_ptr_real_null_ == NULL);
        BOOST_CHECK_EQUAL(configurable_out.shared_ptr_real_null_, configurable_in.shared_ptr_real_null_);
#    endif


#    ifdef ARILES_ADAPTER_BOOST_OPTIONAL
        BOOST_CHECK(configurable_out.optional_real_ != boost::none);
        BOOST_CHECK(configurable_in.optional_real_ != boost::none);
        BOOST_CHECK_CLOSE(*configurable_out.optional_real_, *configurable_in.optional_real_, g_tolerance);
        BOOST_CHECK(configurable_out.optional_real_null_ == boost::none);
        BOOST_CHECK(configurable_out.optional_real_null_ == configurable_in.optional_real_null_);
#    endif
    }
#endif
}  // namespace ariles_tests
