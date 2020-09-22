/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

#include <iostream>
#include <ariles2/types.h>

namespace ariles_tests
{
    class ConfigurablePointers : public ariles2::DefaultBase
    {
    public:
        class MinimalBase : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE

        public:
            bool defaults_check_flag_;
            bool finalize_check_flag_;

        public:
            MinimalBase()
            {
                ariles2::apply<ariles2::Defaults>(*this);
                defaults_check_flag_ = false;
                finalize_check_flag_ = false;
            }

            virtual ~MinimalBase()
            {
            }

#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
            virtual void randomize()
            {
                ARILES2_TRACE_FUNCTION;
                boost::random::random_device random_generator;
                real_ = GET_RANDOM_REAL;
            }
#endif

            void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
            {
                ARILES2_TRACE_FUNCTION;
                real_ = 0.0;
            }
        };


        class Minimal : public MinimalBase
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_PARENT(v, MinimalBase)                                                                                     \
    ARILES2_TYPED_ENTRY_(v, integer_member, int)
#include ARILES2_INITIALIZE

        public:
            Minimal()
            {
                ariles2::apply<ariles2::Defaults>(*this);
                finalize_check_flag_ = false;
            }

            virtual ~Minimal()
            {
            }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
            void randomize()
            {
                ARILES2_TRACE_FUNCTION;
                MinimalBase::randomize();
                boost::random::random_device random_generator;
                integer_member_ = GET_RANDOM_INT;
            }
#endif

            bool operator==(const Minimal &other) const
            {
                return (this->integer_member_ == other.integer_member_);
            }

            const std::string &arilesInstanceID() const
            {
                static std::string instance_id("Minimal");
                return (instance_id);
            }


            void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                arilesVisitParents(visitor, param);
                integer_member_ = 0;
                defaults_check_flag_ = true;
            }

            void arilesVisit(const ariles2::Finalize &visitor, const ariles2::Finalize::Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                arilesVisitParents(visitor, param);
                finalize_check_flag_ = true;
            }
        };



#define ARILES2_DEFAULT_ID "ConfigurablePointers"
#define ARILES2_ENTRIES_0(v)

#if __cplusplus >= 201103L
#    define ARILES2_ENTRIES_1(v)                                                                                       \
        ARILES2_ENTRIES_0(v)                                                                                           \
        ARILES2_TYPED_ENTRY_(v, std_shared_ptr_test, std::shared_ptr<Minimal>)                                         \
        ARILES2_TYPED_ENTRY_(v, std_shared_ptr_test_non_null, ariles2::NonNullPointer<std::shared_ptr<Minimal> >)      \
        ARILES2_TYPED_ENTRY_(v, std_unique_ptr_test, std::unique_ptr<Minimal>)
#else
#    define ARILES2_ENTRIES_1(v) ARILES2_ENTRIES_0(v)
#endif


#ifdef ARILES_ADAPTER_BOOST_POINTER
#    if BOOST_VERSION >= 105800
#        define ARILES2_ENTRIES_2(v)                                                                                   \
            ARILES2_ENTRIES_1(v)                                                                                       \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_test, boost::shared_ptr<Minimal>)                                       \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_test_null, boost::shared_ptr<Minimal>)                                  \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_test_non_null, ariles2::NonNullPointer<boost::shared_ptr<Minimal> >)    \
            ARILES2_TYPED_ENTRY_(v, unique_ptr_test, boost::movelib::unique_ptr<Minimal>)
#    else
#        define ARILES2_ENTRIES_2(v)                                                                                   \
            ARILES2_ENTRIES_1(v)                                                                                       \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_test, boost::shared_ptr<Minimal>)                                       \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_test_non_null, ariles2::NonNullPointer<boost::shared_ptr<Minimal> >)    \
            ARILES2_TYPED_ENTRY_(v, shared_ptr_test_null, boost::shared_ptr<Minimal>)
#    endif
#else
#    define ARILES2_ENTRIES_2(v) ARILES2_ENTRIES_1(v)
#endif


#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
#    define ARILES2_ENTRIES_3(v)                                                                                       \
        ARILES2_ENTRIES_2(v)                                                                                           \
        ARILES2_TYPED_ENTRY_(v, optional_test, boost::optional<Minimal>)                                               \
        ARILES2_TYPED_ENTRY_(v, optional_test_null, boost::optional<Minimal>)
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
        ConfigurablePointers()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
#if __cplusplus >= 201103L
            std_shared_ptr_test_.reset();
            std_unique_ptr_test_.reset();
#endif

#ifdef ARILES_ADAPTER_BOOST_POINTER
            shared_ptr_test_.reset();
            shared_ptr_test_null_.reset();
#    if BOOST_VERSION >= 105800
            unique_ptr_test_.reset();
#    endif
#endif

#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
            optional_test_ = boost::none;
            optional_test_null_ = boost::none;
#endif
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
#    if __cplusplus >= 201103L
            std_shared_ptr_test_ = std::make_shared<Minimal>();
            std_shared_ptr_test_->randomize();

            BOOST_CHECK(false == std_shared_ptr_test_non_null_.isNull());
            std_shared_ptr_test_non_null_->randomize();

            std_unique_ptr_test_.reset(new Minimal());
            std_unique_ptr_test_->randomize();
#    endif

#    ifdef ARILES_ADAPTER_BOOST_POINTER
            shared_ptr_test_ = boost::make_shared<Minimal>();
            shared_ptr_test_->randomize();

            BOOST_CHECK(false == shared_ptr_test_non_null_.isNull());
            shared_ptr_test_non_null_->randomize();

#        if BOOST_VERSION >= 105800
            unique_ptr_test_ = boost::movelib::make_unique<Minimal>();
            unique_ptr_test_->randomize();
#        endif

            shared_ptr_test_null_.reset();
#    endif

#    ifdef ARILES_ADAPTER_BOOST_OPTIONAL
            {
                Minimal minimal;
                minimal.randomize();
                optional_test_ = minimal;
                optional_test_null_ = boost::none;
            }
#    endif
        }
#endif
    };


#ifndef ARILES_TESTS_COMPARE_DISABLED
    template <class t_Configurable_out, class t_Configurable_in>
    void compareMinimal(const t_Configurable_out &configurable_out, const t_Configurable_in &configurable_in)
    {
        BOOST_CHECK_EQUAL(
                dynamic_cast<const ConfigurablePointers::Minimal &>(*configurable_out).integer_member_,
                dynamic_cast<const ConfigurablePointers::Minimal &>(*configurable_in).integer_member_);
        BOOST_CHECK_CLOSE(configurable_out->real_, configurable_in->real_, g_tolerance);
    }

    template <class t_Configurable_out, class t_Configurable_in>
    void compare(const t_Configurable_out &configurable_out, const t_Configurable_in &configurable_in)
    {
#    if __cplusplus >= 201103L
        if (configurable_in.std_shared_ptr_test_ == NULL)
        {
            BOOST_CHECK_EQUAL(configurable_out.std_shared_ptr_test_, configurable_in.std_shared_ptr_test_);
        }
        else
        {
            BOOST_CHECK(configurable_out.std_shared_ptr_test_ != NULL);
            compareMinimal(configurable_out.std_shared_ptr_test_, configurable_in.std_shared_ptr_test_);
        }
        if (configurable_in.std_unique_ptr_test_ == NULL)
        {
            BOOST_CHECK(configurable_out.std_unique_ptr_test_ == configurable_in.std_unique_ptr_test_);
        }
        else
        {
            BOOST_CHECK(configurable_out.std_unique_ptr_test_ != NULL);
            compareMinimal(configurable_out.std_unique_ptr_test_, configurable_in.std_unique_ptr_test_);
        }

        BOOST_CHECK(false == configurable_in.std_shared_ptr_test_non_null_.isNull());
        BOOST_CHECK(false == configurable_out.std_shared_ptr_test_non_null_.isNull());
        compareMinimal(configurable_out.std_shared_ptr_test_non_null_, configurable_in.std_shared_ptr_test_non_null_);
        BOOST_CHECK(configurable_in.std_shared_ptr_test_non_null_->defaults_check_flag_);
        BOOST_CHECK(configurable_in.std_shared_ptr_test_non_null_->finalize_check_flag_);
#    endif


#    ifdef ARILES_ADAPTER_BOOST_POINTER
        if (configurable_in.shared_ptr_test_ == NULL)
        {
            BOOST_CHECK_EQUAL(configurable_out.shared_ptr_test_, configurable_in.shared_ptr_test_);
        }
        else
        {
            BOOST_CHECK(configurable_out.shared_ptr_test_ != NULL);
            compareMinimal(configurable_out.shared_ptr_test_, configurable_in.shared_ptr_test_);
            BOOST_CHECK(configurable_in.shared_ptr_test_->defaults_check_flag_);
            BOOST_CHECK(configurable_in.shared_ptr_test_->finalize_check_flag_);
        }
#        if BOOST_VERSION >= 105800
        if (configurable_in.unique_ptr_test_ == NULL)
        {
            BOOST_CHECK(configurable_out.unique_ptr_test_ == configurable_in.unique_ptr_test_);
        }
        else
        {
            BOOST_CHECK(configurable_out.unique_ptr_test_ != NULL);
            compareMinimal(configurable_out.unique_ptr_test_, configurable_in.unique_ptr_test_);
            BOOST_CHECK(configurable_in.unique_ptr_test_->defaults_check_flag_);
            BOOST_CHECK(configurable_in.unique_ptr_test_->finalize_check_flag_);
        }
#        endif

        BOOST_CHECK(configurable_out.shared_ptr_test_null_ == NULL);
        BOOST_CHECK_EQUAL(configurable_out.shared_ptr_test_null_, configurable_in.shared_ptr_test_null_);

        BOOST_CHECK(false == configurable_in.shared_ptr_test_non_null_.isNull());
        BOOST_CHECK(false == configurable_out.shared_ptr_test_non_null_.isNull());
        compareMinimal(configurable_out.shared_ptr_test_non_null_, configurable_in.shared_ptr_test_non_null_);
        BOOST_CHECK(configurable_in.shared_ptr_test_non_null_->defaults_check_flag_);
        BOOST_CHECK(configurable_in.shared_ptr_test_non_null_->finalize_check_flag_);
#    endif


#    ifdef ARILES_ADAPTER_BOOST_OPTIONAL
        BOOST_CHECK(configurable_out.optional_test_ != boost::none);
        BOOST_CHECK(configurable_in.optional_test_ != boost::none);
        compareMinimal(configurable_out.optional_test_, configurable_in.optional_test_);
        BOOST_CHECK(configurable_out.optional_test_null_ == boost::none);
        BOOST_CHECK(configurable_in.optional_test_null_ == boost::none);
        BOOST_CHECK(configurable_in.optional_test_->finalize_check_flag_);
#    endif
    }
#endif
}  // namespace ariles_tests
