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
    class ConfigurablePointers : public ariles::ConfigurableBase
    {
        public:
            class Minimal : public ariles::ConfigurableBase
            {
                #define ARILES_SECTION_ID "ConfigurableEntryName"
                #define ARILES_AUTO_DEFAULTS
                #define ARILES_ENTRIES \
                    ARILES_TYPED_ENTRY_(integer_member, int)
                #include ARILES_INITIALIZE

                public:
                    virtual ~Minimal()
                    {
                    }

                    bool operator==(const Minimal &other) const
                    {
                        return (this->integer_member_ == other.integer_member_);
                    }
            };


        #define ARILES_SECTION_ID "ConfigurablePointers"
        #define ARILES_CONSTRUCTOR ConfigurablePointers


        #define ARILES_ENTRIES_0

#if __cplusplus >= 201103L
        #define ARILES_ENTRIES_1 \
            ARILES_ENTRIES_0 \
            ARILES_TYPED_ENTRY_(std_shared_ptr_test,          std::shared_ptr<Minimal>) \
            ARILES_TYPED_ENTRY_(std_shared_ptr_test_non_null, ariles::NonNullPointer<std::shared_ptr<Minimal> >) \
            ARILES_TYPED_ENTRY_(std_unique_ptr_test,          std::unique_ptr<Minimal>)
#else
        #define ARILES_ENTRIES_1 ARILES_ENTRIES_0
#endif


#ifdef ARILES_ADAPTER_BOOST_POINTER
#   if BOOST_VERSION >= 105800
#       define ARILES_ENTRIES_2 \
                ARILES_ENTRIES_1 \
                ARILES_TYPED_ENTRY_(shared_ptr_test,          boost::shared_ptr<Minimal>) \
                ARILES_TYPED_ENTRY_(shared_ptr_test_null,     boost::shared_ptr<Minimal>) \
                ARILES_TYPED_ENTRY_(shared_ptr_test_non_null, ariles::NonNullPointer<boost::shared_ptr<Minimal> >) \
                ARILES_TYPED_ENTRY_(unique_ptr_test,          boost::movelib::unique_ptr<Minimal>)
#   else
#       define ARILES_ENTRIES_2 \
                ARILES_ENTRIES_1 \
                ARILES_TYPED_ENTRY_(shared_ptr_test,          boost::shared_ptr<Minimal>) \
                ARILES_TYPED_ENTRY_(shared_ptr_test_non_null, ariles::NonNullPointer<boost::shared_ptr<Minimal> >) \
                ARILES_TYPED_ENTRY_(shared_ptr_test_null,     boost::shared_ptr<Minimal>)
#   endif
#else
        #define ARILES_ENTRIES_2 ARILES_ENTRIES_1
#endif


#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
        #define ARILES_ENTRIES_3 \
            ARILES_ENTRIES_2 \
            ARILES_TYPED_ENTRY_(optional_test,          boost::optional<Minimal>) \
            ARILES_TYPED_ENTRY_(optional_test_null,     boost::optional<Minimal>)
#else
        #define ARILES_ENTRIES_3 ARILES_ENTRIES_2
#endif


        #define ARILES_ENTRIES ARILES_ENTRIES_3
        #include ARILES_INITIALIZE

#undef ARILES_ENTRIES_0
#undef ARILES_ENTRIES_1
#undef ARILES_ENTRIES_2
#undef ARILES_ENTRIES_3


        public:
            ConfigurablePointers()
            {
                setDefaults();
            }

            virtual void setDefaults()
            {

#if __cplusplus >= 201103L
                std_shared_ptr_test_.reset();
                std_unique_ptr_test_.reset();
#endif

#ifdef ARILES_ADAPTER_BOOST_POINTER
                shared_ptr_test_.reset();
                shared_ptr_test_null_.reset();
#   if BOOST_VERSION >= 105800
                unique_ptr_test_.reset();
#   endif
#endif

#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
                optional_test_ = boost::none;
                optional_test_null_ = boost::none;
#endif
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;
#   if __cplusplus >= 201103L
                std_shared_ptr_test_ = std::make_shared<Minimal>();
                std_shared_ptr_test_->integer_member_ = GET_RANDOM_INT;

                BOOST_CHECK(false == std_shared_ptr_test_non_null_.isNull());
                std_shared_ptr_test_non_null_->integer_member_ = GET_RANDOM_INT;

                std_unique_ptr_test_.reset(new Minimal());
                std_unique_ptr_test_->integer_member_ = GET_RANDOM_INT;
#   endif

#   ifdef ARILES_ADAPTER_BOOST_POINTER
                shared_ptr_test_ = boost::make_shared<Minimal>();
                shared_ptr_test_->integer_member_ = GET_RANDOM_INT;

                BOOST_CHECK(false == shared_ptr_test_non_null_.isNull());
                shared_ptr_test_non_null_->integer_member_ = GET_RANDOM_INT;

#       if BOOST_VERSION >= 105800
                unique_ptr_test_ = boost::movelib::make_unique<Minimal>();
                unique_ptr_test_->integer_member_ = GET_RANDOM_INT;
#       endif

                shared_ptr_test_null_.reset();
#   endif

#   ifdef ARILES_ADAPTER_BOOST_OPTIONAL
                {
                    Minimal minimal;
                    minimal.integer_member_ = GET_RANDOM_INT;
                    optional_test_ = minimal;
                    optional_test_null_ = boost::none;
                }
#   endif
            }
#endif
    };


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
    template<class t_Configurable_out, class t_Configurable_in>
    void    compare(const t_Configurable_out    &configurable_out,
                    const t_Configurable_in     &configurable_in)
    {
#if __cplusplus >= 201103L
        if (configurable_in.std_shared_ptr_test_ == NULL)
        {
            BOOST_CHECK_EQUAL(configurable_out.std_shared_ptr_test_, configurable_in.std_shared_ptr_test_);
        }
        else
        {
            BOOST_CHECK(configurable_out.std_shared_ptr_test_ != NULL);
            BOOST_CHECK_EQUAL(  configurable_out.std_shared_ptr_test_->integer_member_,
                                configurable_in.std_shared_ptr_test_->integer_member_);
        }
        if (configurable_in.std_unique_ptr_test_ == NULL)
        {
            BOOST_CHECK(configurable_out.std_unique_ptr_test_ == configurable_in.std_unique_ptr_test_);
        }
        else
        {
            BOOST_CHECK(configurable_out.std_unique_ptr_test_ != NULL);
            BOOST_CHECK_EQUAL(  configurable_out.std_unique_ptr_test_->integer_member_,
                                configurable_in.std_unique_ptr_test_->integer_member_);
        }

        BOOST_CHECK(false == configurable_in.std_shared_ptr_test_non_null_.isNull());
        BOOST_CHECK(false == configurable_out.std_shared_ptr_test_non_null_.isNull());
        BOOST_CHECK_EQUAL(  configurable_out.std_shared_ptr_test_non_null_->integer_member_,
                            configurable_in.std_shared_ptr_test_non_null_->integer_member_);
#endif


#ifdef ARILES_ADAPTER_BOOST_POINTER
        if (configurable_in.shared_ptr_test_ == NULL)
        {
            BOOST_CHECK_EQUAL(configurable_out.shared_ptr_test_, configurable_in.shared_ptr_test_);
        }
        else
        {
            BOOST_CHECK(configurable_out.shared_ptr_test_ != NULL);
            BOOST_CHECK_EQUAL(  configurable_out.shared_ptr_test_->integer_member_,
                                configurable_in.shared_ptr_test_->integer_member_);
        }
#   if BOOST_VERSION >= 105800
        if (configurable_in.unique_ptr_test_ == NULL)
        {
            BOOST_CHECK(configurable_out.unique_ptr_test_ == configurable_in.unique_ptr_test_);
        }
        else
        {
            BOOST_CHECK(configurable_out.unique_ptr_test_ != NULL);
            BOOST_CHECK_EQUAL(  configurable_out.unique_ptr_test_->integer_member_,
                                configurable_in.unique_ptr_test_->integer_member_);
        }
#   endif

        BOOST_CHECK(configurable_out.shared_ptr_test_null_ == NULL);
        BOOST_CHECK_EQUAL(configurable_out.shared_ptr_test_null_, configurable_in.shared_ptr_test_null_);

        BOOST_CHECK(false == configurable_in.shared_ptr_test_non_null_.isNull());
        BOOST_CHECK(false == configurable_out.shared_ptr_test_non_null_.isNull());
        BOOST_CHECK_EQUAL(  configurable_out.shared_ptr_test_non_null_->integer_member_,
                            configurable_in.shared_ptr_test_non_null_->integer_member_);
#endif


#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
        BOOST_CHECK(configurable_out.optional_test_ != boost::none);
        BOOST_CHECK(configurable_in.optional_test_ != boost::none);
        BOOST_CHECK_EQUAL(  configurable_out.optional_test_->integer_member_,
                            configurable_in.optional_test_->integer_member_);
        BOOST_CHECK(configurable_out.optional_test_null_ == boost::none);
        BOOST_CHECK(configurable_in.optional_test_null_ == boost::none);
#endif
    }
#endif
}
