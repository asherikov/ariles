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
    class AbstractNonNullPointer : public ariles2::DefaultBase
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

            void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
            {
                ARILES2_TRACE_FUNCTION;
                real_ = 123.4;
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
                integer_member_ = 414;
                defaults_check_flag_ = true;
            }
        };



#define ARILES2_DEFAULT_ID "AbstractNonNullPointer"
#define ARILES2_ENTRIES_0(v)

#if __cplusplus >= 201103L
#    define ARILES2_ENTRIES_1(v)                                                                                       \
        ARILES2_ENTRIES_0(v)                                                                                           \
        ARILES2_TYPED_ENTRY_(v, std_shared_ptr_test_non_null, ariles2::NonNullPointer<std::shared_ptr<MinimalBase> >)
#else
#    define ARILES2_ENTRIES_1(v) ARILES2_ENTRIES_0(v)
#endif


#ifdef ARILES_ADAPTER_BOOST_POINTER
#    define ARILES2_ENTRIES_2(v)                                                                                       \
        ARILES2_ENTRIES_1(v)                                                                                           \
        ARILES2_TYPED_ENTRY_(v, shared_ptr_test_non_null, ariles2::NonNullPointer<boost::shared_ptr<MinimalBase> >)
#else
#    define ARILES2_ENTRIES_2(v) ARILES2_ENTRIES_1(v)
#endif


#define ARILES2_ENTRIES(v) ARILES2_ENTRIES_2(v)
#include ARILES2_INITIALIZE

#undef ARILES2_ENTRIES_0
#undef ARILES2_ENTRIES_1
#undef ARILES2_ENTRIES_2

    public:
        AbstractNonNullPointer()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
#if __cplusplus >= 201103L
            BOOST_CHECK(false == std_shared_ptr_test_non_null_.isNull());
            std_shared_ptr_test_non_null_ = std::make_shared<Minimal>();
#endif

#ifdef ARILES_ADAPTER_BOOST_POINTER
            BOOST_CHECK(false == shared_ptr_test_non_null_.isNull());
            shared_ptr_test_non_null_ = boost::make_shared<Minimal>();
#endif
        }
    };
}  // namespace ariles_tests
