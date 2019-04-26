/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once

namespace ariles_tests
{
    class Base : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "Base"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(real, double)
        #define ARILES_AUTO_DEFAULTS
        #include ARILES_INITIALIZE

        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            virtual void randomize()
            {
                real_ = GET_RANDOM_REAL;
            }
#endif

            Base()
            {
                setDefaults();
            }

            virtual ~Base()
            {
            }
    };


    class Derived1 : public Base
    {
        #define ARILES_SECTION_ID "Derived1"
        #define ARILES_ENTRIES \
            ARILES_PARENT(Base) \
            ARILES_TYPED_ENTRY_(real1, double)
        #define ARILES_AUTO_DEFAULTS
        #include ARILES_INITIALIZE

        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                Base::randomize();
                real1_ = GET_RANDOM_REAL;
            }
#endif

            Derived1()
            {
                setDefaults();
            }
    };


    class Derived2 : public Base
    {
        #define ARILES_SECTION_ID "Derived2"
        #define ARILES_ENTRIES \
            ARILES_PARENT(Base) \
            ARILES_TYPED_ENTRY_(real2, double)
        #define ARILES_AUTO_DEFAULTS
        #include ARILES_INITIALIZE

        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                Base::randomize();
                real2_ = GET_RANDOM_REAL;
            }
#endif

            Derived2()
            {
                setDefaults();
            }
    };


    template<template<class> class t_Pointer, class t_Instantiator>
    class CommonAny : public ariles::Any<t_Pointer, Base, t_Instantiator>
    {
        public:
#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                BOOST_CHECK(false == this->isInitialized());

                this->id_ = "test";
                BOOST_CHECK(false == this->isInitialized());

                BOOST_CHECK_THROW(this->build("test"), std::exception);

                BOOST_CHECK_NO_THROW(this->build("Derived1"));
                BOOST_CHECK(true == this->isInitialized());

                BOOST_CHECK(NULL != this->template cast<Derived1>());
                BOOST_CHECK(NULL != this->template cast<Derived1>("Derived1"));
                BOOST_CHECK(NULL == this->template cast<Derived1>("Derived2"));

                double test = 0.0;

                this->operator->()->real_ = 10.0;
                test = this->operator->()->real_;

                this->operator*().real_ = 10.0;
                test = this->operator*().real_;

                this->template cast<Derived1>()->real1_ = 10.0;
                test = this->template cast<Derived1>()->real1_;


                test = GET_RANDOM_REAL;
                if(test > 0.0)
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


#if __cplusplus >= 201103L
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
#endif


    class ConfigurableAny : public ariles::ConfigurableBase
    {
        public:
            #define ARILES_SECTION_ID "ConfigurableAny"
            #define ARILES_CONSTRUCTOR ConfigurableAny
            #define ARILES_AUTO_DEFAULTS


            #define ARILES_ENTRIES_0

#if __cplusplus >= 201103L
            CommonAny<std::shared_ptr, StdPtrInstantiator> std_any_;

            #define ARILES_ENTRIES_1 \
                ARILES_ENTRIES_0 \
                ARILES_ENTRY_(std_any)
#else
            #define ARILES_ENTRIES_1 ARILES_ENTRIES_0
#endif


#ifdef ARILES_ADAPTER_BOOST_POINTER
            CommonAny<boost::shared_ptr, BoostPtrInstantiator> boost_any_;
            #define ARILES_ENTRIES_2 \
                    ARILES_ENTRIES_1 \
                    ARILES_ENTRY_(boost_any)
#else
            #define ARILES_ENTRIES_2 ARILES_ENTRIES_1
#endif


            #define ARILES_ENTRIES ARILES_ENTRIES_2
            #include ARILES_INITIALIZE

#undef ARILES_ENTRIES_0
#undef ARILES_ENTRIES_1
#undef ARILES_ENTRIES_2


        public:
            ConfigurableAny()
            {
                setDefaults();
            }

#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
#if __cplusplus >= 201103L
                std_any_.randomize();
#endif

#ifdef ARILES_ADAPTER_BOOST_POINTER
                boost_any_.randomize();
#endif
            }
#endif
    };


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
    template<class t_Configurable_out, class t_Configurable_in>
    void    compare(const t_Configurable_out    &configurable_out,
                    const t_Configurable_in     &configurable_in)
    {
#if __cplusplus >= 201103L
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
#endif


#ifdef ARILES_ADAPTER_BOOST_POINTER
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
#endif
    }
#endif
}
