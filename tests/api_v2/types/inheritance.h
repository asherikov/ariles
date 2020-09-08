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
    template <class t_Scalar>
    class ConfigurableMember : virtual public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, t_Scalar)                                                                         \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE


    public:
        ConfigurableMember()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        virtual ~ConfigurableMember()
        {
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            integer_ = 10;
            real_ = 1.33;
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            integer_ = GET_RANDOM_INT;
            real_ = GET_RANDOM_REAL;
            ariles2::apply<ariles2::PostRead>(*this);
        }
#endif
    };


    template <class t_Scalar>
    class ConfigurableMember1 : public ConfigurableMember<t_Scalar>
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, member, ConfigurableMember<t_Scalar>)
#include ARILES2_INITIALIZE


    public:
        ConfigurableMember1()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        virtual ~ConfigurableMember1()
        {
        }


        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            visitor.visitMapEntry(member_, "member", param);
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            member_.randomize();
            ariles2::apply<ariles2::PostRead>(*this);
        }
#endif
    };


    class ConfigurableBase : virtual public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, member, ConfigurableMember<int>)
#include ARILES2_INITIALIZE


    public:
        ConfigurableBase()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        virtual ~ConfigurableBase()
        {
        }


        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            visitor.visitMapEntry(member_, "member", param);
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            member_.randomize();
            ariles2::apply<ariles2::PostRead>(*this);
        }
#endif
    };


    class ConfigurableDerived : public ConfigurableBase, public ConfigurableMember<int>
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_PARENT(v, ConfigurableBase)                                                                                \
    ARILES2_PARENT(v, ConfigurableMember<int>)                                                                         \
    ARILES2_TYPED_ENTRY_(v, another_member, ConfigurableMember<int>)                                                   \
    ARILES2_TYPED_ENTRY_(v, another_member1, ConfigurableMember1<int>)
#include ARILES2_INITIALIZE


    public:
        ConfigurableDerived()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            visitor.visitMapEntry(another_member_, "another_member", param);
            visitor.visitMapEntry(another_member1_, "another_member1", param);
            ConfigurableBase::arilesVisit(visitor, param);
            ConfigurableMember<int>::arilesVisit(visitor, param);
        }


#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            boost::random::random_device random_generator;
            another_member_.randomize();
            another_member1_.randomize();
            ConfigurableBase::randomize();
            ConfigurableMember<int>::randomize();
            ariles2::apply<ariles2::PostRead>(*this);
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

        BOOST_CHECK_EQUAL(configurable_out.another_member_.integer_, configurable_in.another_member_.integer_);
        BOOST_CHECK_CLOSE(configurable_out.another_member_.real_, configurable_in.another_member_.real_, g_tolerance);

        BOOST_CHECK_EQUAL(configurable_out.member_.integer_, configurable_in.member_.integer_);
        BOOST_CHECK_CLOSE(configurable_out.member_.real_, configurable_in.member_.real_, g_tolerance);
    }
#endif
}  // namespace ariles_tests
