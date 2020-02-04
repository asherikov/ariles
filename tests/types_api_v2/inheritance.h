/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    template<class t_Scalar>
    class ConfigurableMember : virtual public ariles::Base
    {
        #define ARILES_DEFAULT_ID "ConfigurableMember"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer, t_Scalar) \
            ARILES_TYPED_ENTRY_(real   , double)
        #include ARILES_INITIALIZE


        public:
            ConfigurableMember()
            {
                ariles<ariles::defaults::Visitor>();
            }

            virtual ~ConfigurableMember() {}


            void arilesVisit(   const ariles::defaults::Visitor &/*visitor*/,
                                const ariles::defaults::Visitor::Parameters &/*param*/)
            {
                integer_ = 10;
                real_ = 1.33;
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;
                integer_ = GET_RANDOM_INT;
                real_    = GET_RANDOM_REAL;
                ariles<ariles::finalize::Visitor>();
            }
#endif
    };


    template<class t_Scalar>
    class ConfigurableMember1 : public ConfigurableMember<t_Scalar>
    {
        #define ARILES_DEFAULT_ID "ConfigurableMember1"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(member,         ConfigurableMember<t_Scalar>)
        #include ARILES_INITIALIZE


        public:
            ConfigurableMember1()
            {
                ariles<ariles::defaults::Visitor>();
            }

            virtual ~ConfigurableMember1() {}


            void arilesVisit(   const ariles::defaults::Visitor &visitor,
                                const ariles::defaults::Visitor::Parameters &param)
            {
                ariles::defaults::arilesEntryApply(visitor, member_, "member", param);
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;
                member_.randomize();
                this->ariles<ariles::finalize::Visitor>();
            }
#endif
    };


    class ConfigurableBase : virtual public ariles::Base
    {
        #define ARILES_DEFAULT_ID "ConfigurableBase"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(member,         ConfigurableMember<int>)
        #include ARILES_INITIALIZE


        public:
            ConfigurableBase()
            {
                ariles<ariles::defaults::Visitor>();
            }

            virtual ~ConfigurableBase() {}


            void arilesVisit(   const ariles::defaults::Visitor &visitor,
                                const ariles::defaults::Visitor::Parameters &param)
            {
                ariles::defaults::arilesEntryApply(visitor, member_, "member", param);
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;
                member_.randomize();
                ariles<ariles::finalize::Visitor>();
            }
#endif
    };


    class ConfigurableDerived : public ConfigurableBase, public ConfigurableMember<int>
    {
        #define ARILES_DEFAULT_ID "ConfigurableDerived"
        #define ARILES_ENTRIES \
            ARILES_PARENT(ConfigurableBase) \
            ARILES_PARENT(ConfigurableMember<int>) \
            ARILES_TYPED_ENTRY_(another_member,         ConfigurableMember<int>) \
            ARILES_TYPED_ENTRY_(another_member1,        ConfigurableMember1<int>)
        #include ARILES_INITIALIZE


        public:
            ConfigurableDerived()
            {
                ariles<ariles::defaults::Visitor>();
            }


            void arilesVisit(   const ariles::defaults::Visitor &visitor,
                                const ariles::defaults::Visitor::Parameters &param)
            {
                ariles::defaults::arilesEntryApply(visitor, another_member_, "another_member", param);
                ariles::defaults::arilesEntryApply(visitor, another_member1_, "another_member1", param);
                ConfigurableBase::arilesVisit(visitor, param);
                ConfigurableMember<int>::arilesVisit(visitor, param);
            }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;
                another_member_.randomize();
                another_member1_.randomize();
                ConfigurableBase::randomize();
                ConfigurableMember<int>::randomize();
                ariles<ariles::finalize::Visitor>();
            }
#endif
    };


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
    // comparison
    template<class t_Configurable_out, class t_Configurable_in>
    void    compare(const t_Configurable_out    &configurable_out,
                    const t_Configurable_in     &configurable_in)
    {
        BOOST_CHECK_EQUAL(configurable_out.integer_,                 configurable_in.integer_);
        BOOST_CHECK_CLOSE(configurable_out.real_,                    configurable_in.real_, g_tolerance);

        BOOST_CHECK_EQUAL(configurable_out.another_member_.integer_, configurable_in.another_member_.integer_);
        BOOST_CHECK_CLOSE(configurable_out.another_member_.real_,    configurable_in.another_member_.real_, g_tolerance);

        BOOST_CHECK_EQUAL(configurable_out.member_.integer_,         configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(configurable_out.member_.real_,            configurable_in.member_.real_, g_tolerance);
    }
#endif
}
