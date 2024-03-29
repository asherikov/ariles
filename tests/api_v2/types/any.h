/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

#include <ariles2/types.h>

namespace ariles_tests
{
    class Base : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE

    public:
        bool defaults_check_flag_;
        bool finalize_check_flag_;

    public:
#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        virtual void randomize()
        {
            boost::random::random_device random_generator;
            real_ = GET_RANDOM_REAL;
        }
#endif

        Base()
        {
            ariles2::apply<ariles2::Defaults>(*this);
            defaults_check_flag_ = false;
            finalize_check_flag_ = false;
        }

        virtual ~Base()
        {
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            real_ = 0.0;
        }
    };


    class Derived1 : public Base
    {
#define ARILES2_DEFAULT_ID "Derived1"  // Needed for cast<Derived1>("Derived1"));
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_PARENT(v, Base)                                                                                            \
    ARILES2_TYPED_ENTRY_(v, real1, double)
#include ARILES2_INITIALIZE

    public:
#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize() override
        {
            Base::randomize();
            boost::random::random_device random_generator;
            real1_ = GET_RANDOM_REAL;
        }
#endif

        Derived1()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            arilesVisit<ariles2::Defaults>(visitor, param);
            defaults_check_flag_ = true;
        }

        void arilesVisit(const ariles2::Finalize &visitor, const ariles2::Finalize::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            arilesVisit<ariles2::Finalize>(visitor, param);
            finalize_check_flag_ = true;
        }
    };


    class Derived2 : public Base
    {
#define ARILES2_DEFAULT_ID "Derived2"  // Needed for cast<Derived2>("Derived2"));
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_PARENT(v, Base)                                                                                            \
    ARILES2_TYPED_ENTRY_(v, real2, double)
#include ARILES2_INITIALIZE

    public:
#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize() override
        {
            Base::randomize();
            boost::random::random_device random_generator;
            real2_ = GET_RANDOM_REAL;
        }
#endif

        Derived2()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            arilesVisit<ariles2::Defaults>(visitor, param);
            defaults_check_flag_ = true;
        }

        void arilesVisit(const ariles2::Finalize &visitor, const ariles2::Finalize::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            arilesVisit<ariles2::Finalize>(visitor, param);
            finalize_check_flag_ = true;
        }
    };


    template <template <class> class t_Pointer, class t_Instantiator>
    class CommonAny : public ariles2::Any<t_Pointer, Base, t_Instantiator>
    {
    public:
#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            BOOST_CHECK(not this->isInitialized());

            this->id_ = "test";
            BOOST_CHECK(not this->isInitialized());

            BOOST_CHECK_THROW(this->build("test"), std::exception);

            BOOST_CHECK_NO_THROW(this->build("Derived1"));
            BOOST_CHECK(this->isInitialized());

            BOOST_CHECK(NULL != this->template cast<Derived1>());
            BOOST_CHECK(NULL != this->template cast<Derived1>("Derived1"));
            BOOST_CHECK(NULL == this->template cast<Derived1>("Derived2"));

            double test = 0.0;

            this->operator->()->real_ = 10.0;
            test = this->operator->()->real_;
            BOOST_CHECK_EQUAL(test, 10.0);

            this->operator*().real_ = 10.0;
            test = this->operator*().real_;
            BOOST_CHECK_EQUAL(test, 10.0);

            this->template cast<Derived1>()->real1_ = 10.0;
            test = this->template cast<Derived1>()->real1_;
            BOOST_CHECK_EQUAL(test, 10.0);


            test = GET_RANDOM_REAL;
            if (test > 0.0)
            {
                this->build("Derived1");
            }
            else
            {
                this->build("Derived2");
            }

            this->value_->randomize();
        }
#endif
    };


#ifdef ARILES_ADAPTER_BOOST_POINTER
    class BoostPtrInstantiator
    {
    public:
        static boost::shared_ptr<Base> instantiate(const std::string &id)
        {
            if (id == "Derived1")
            {
                return (boost::make_shared<Derived1>());
            }
            if (id == "Derived2")
            {
                return (boost::make_shared<Derived2>());
            }

            return (boost::shared_ptr<Base>());
        }
    };
#endif


    class StdPtrInstantiator
    {
    public:
        static std::shared_ptr<Base> instantiate(const std::string &id)
        {
            if (id == "Derived1")
            {
                return (std::make_shared<Derived1>());
            }
            if (id == "Derived2")
            {
                return (std::make_shared<Derived2>());
            }

            return (std::shared_ptr<Base>());
        }
    };


    class ConfigurableAny : public ariles2::DefaultBase
    {
    public:
#define ARILES2_ENTRIES_0(v)

        CommonAny<std::shared_ptr, StdPtrInstantiator> std_any_;

#define ARILES2_ENTRIES_1(v)                                                                                           \
    ARILES2_ENTRIES_0(v)                                                                                               \
    ARILES2_ENTRY_(v, std_any)


#ifdef ARILES_ADAPTER_BOOST_POINTER
        CommonAny<boost::shared_ptr, BoostPtrInstantiator> boost_any_;
#    define ARILES2_ENTRIES_2(v)                                                                                       \
        ARILES2_ENTRIES_1(v)                                                                                           \
        ARILES2_ENTRY_(v, boost_any)
#else
#    define ARILES2_ENTRIES_2(v) ARILES2_ENTRIES_1(v)
#endif


#define ARILES2_ENTRIES(v) ARILES2_ENTRIES_2(v)
#include ARILES2_INITIALIZE

#undef ARILES2_ENTRIES_0
#undef ARILES2_ENTRIES_1
#undef ARILES2_ENTRIES_2


    public:
        ConfigurableAny()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            std_any_.randomize();

#    ifdef ARILES_ADAPTER_BOOST_POINTER
            boost_any_.randomize();
#    endif
        }
#endif
    };


#ifndef ARILES_TESTS_COMPARE_DISABLED
    template <class t_Configurable_out, class t_Configurable_in>
    void compare(const t_Configurable_out &configurable_out, const t_Configurable_in &configurable_in)
    {
        BOOST_CHECK_EQUAL(configurable_out.std_any_.id_, configurable_in.std_any_.id_);
        BOOST_CHECK_CLOSE((*configurable_out.std_any_).real_, (*configurable_in.std_any_).real_, g_tolerance);
        if ("Derived1" == configurable_out.std_any_.id_)
        {
            BOOST_CHECK_CLOSE(
                    configurable_out.std_any_.template cast<Derived1>()->real1_,
                    configurable_in.std_any_.template cast<Derived1>()->real1_,
                    g_tolerance);
            BOOST_CHECK_CLOSE(
                    configurable_out.std_any_.template cast<Derived1>("Derived1")->real1_,
                    configurable_in.std_any_.template cast<Derived1>("Derived1")->real1_,
                    g_tolerance);
        }
        if ("Derived2" == configurable_out.std_any_.id_)
        {
            BOOST_CHECK_CLOSE(
                    configurable_out.std_any_.template cast<Derived2>()->real2_,
                    configurable_in.std_any_.template cast<Derived2>()->real2_,
                    g_tolerance);
            BOOST_CHECK_CLOSE(
                    configurable_out.std_any_.template cast<Derived2>("Derived2")->real2_,
                    configurable_in.std_any_.template cast<Derived2>("Derived2")->real2_,
                    g_tolerance);
        }
        BOOST_CHECK(configurable_in.std_any_->defaults_check_flag_);
        BOOST_CHECK(configurable_in.std_any_->finalize_check_flag_);


#    ifdef ARILES_ADAPTER_BOOST_POINTER
        BOOST_CHECK_EQUAL(configurable_out.boost_any_.id_, configurable_in.boost_any_.id_);
        BOOST_CHECK_CLOSE((*configurable_out.boost_any_).real_, (*configurable_in.boost_any_).real_, g_tolerance);
        if ("Derived1" == configurable_out.boost_any_.id_)
        {
            BOOST_CHECK_CLOSE(
                    configurable_out.boost_any_.template cast<Derived1>()->real1_,
                    configurable_in.boost_any_.template cast<Derived1>()->real1_,
                    g_tolerance);
            BOOST_CHECK_CLOSE(
                    configurable_out.boost_any_.template cast<Derived1>("Derived1")->real1_,
                    configurable_in.boost_any_.template cast<Derived1>("Derived1")->real1_,
                    g_tolerance);
        }
        if ("Derived2" == configurable_out.boost_any_.id_)
        {
            BOOST_CHECK_CLOSE(
                    configurable_out.boost_any_.template cast<Derived2>()->real2_,
                    configurable_in.boost_any_.template cast<Derived2>()->real2_,
                    g_tolerance);
            BOOST_CHECK_CLOSE(
                    configurable_out.boost_any_.template cast<Derived2>("Derived2")->real2_,
                    configurable_in.boost_any_.template cast<Derived2>("Derived2")->real2_,
                    g_tolerance);
        }
        BOOST_CHECK(configurable_in.boost_any_->defaults_check_flag_);
        BOOST_CHECK(configurable_in.boost_any_->finalize_check_flag_);
#    endif
    }
#endif
}  // namespace ariles_tests
