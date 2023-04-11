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
    class CopyPointers
    {
    public:
        class Minimal
        {
        public:
            double real;
            int integer_member;

        public:
            virtual ~Minimal()
            {
            }

            bool operator==(const Minimal &other) const
            {
                return (this->integer_member == other.integer_member);
            }
        };


    public:
        std::shared_ptr<Minimal> std_shared_ptr_test;
        std::shared_ptr<Minimal> std_shared_ptr_test_non_null;
        std::unique_ptr<Minimal> std_unique_ptr_test;


#ifdef ARILES_ADAPTER_BOOST_POINTER
#    if BOOST_VERSION >= 105800
        boost::shared_ptr<Minimal> shared_ptr_test;
        boost::shared_ptr<Minimal> shared_ptr_test_null;
        boost::shared_ptr<Minimal> shared_ptr_test_non_null;
        boost::movelib::unique_ptr<Minimal> unique_ptr_test;
#    else
        boost::shared_ptr<Minimal> shared_ptr_test;
        boost::shared_ptr<Minimal> shared_ptr_test_non_null;
        boost::shared_ptr<Minimal> shared_ptr_test_null;
#    endif
#endif


#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
        boost::optional<Minimal> optional_test;
        boost::optional<Minimal> optional_test_null;
#endif

    public:
        CopyPointers()
        {
        }
    };
}  // namespace ariles_tests
