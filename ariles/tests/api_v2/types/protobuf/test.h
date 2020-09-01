/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once


namespace ariles_tests
{
    namespace protobuf
    {
        class Scalars : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, test_string, std::string)                                                                  \
    ARILES2_TYPED_ENTRY_(v, test_double, double)                                                                       \
    ARILES2_TYPED_ENTRY_(v, test_float, float)                                                                         \
    ARILES2_TYPED_ENTRY_(v, test_int32, int32_t)                                                                       \
    ARILES2_TYPED_ENTRY_(v, test_sint32, int32_t)                                                                      \
    ARILES2_TYPED_ENTRY_(v, test_sfixed32, int32_t)                                                                    \
    ARILES2_TYPED_ENTRY_(v, test_fixed32, uint32_t)                                                                    \
    ARILES2_TYPED_ENTRY_(v, test_uint32, uint32_t)                                                                     \
    ARILES2_TYPED_ENTRY_(v, test_int64, int64_t)                                                                       \
    ARILES2_TYPED_ENTRY_(v, test_sint64, int64_t)                                                                      \
    ARILES2_TYPED_ENTRY_(v, test_sfixed64, int64_t)                                                                    \
    ARILES2_TYPED_ENTRY_(v, test_uint64, uint64_t)                                                                     \
    ARILES2_TYPED_ENTRY_(v, test_fixed64, uint64_t)                                                                    \
    ARILES2_TYPED_ENTRY_(v, test_bool, bool)                                                                           \
    ARILES2_TYPED_ENTRY_(v, test_bytes, std::string)
#include ARILES2_INITIALIZE

        public:
            virtual ~Scalars()
            {
            }

#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
            void randomize()
            {
                boost::random::random_device random_generator;

                test_string_ = "string";
                test_double_ = GET_RANDOM_REAL;
                test_float_ = GET_RANDOM_REAL;
                test_int32_ = GET_RANDOM_INT;
                test_sint32_ = GET_RANDOM_INT;
                test_sfixed32_ = GET_RANDOM_INT;
                test_fixed32_ = GET_RANDOM_INT;
                test_uint32_ = GET_RANDOM_INT;
                test_int64_ = GET_RANDOM_INT;
                test_sint64_ = GET_RANDOM_INT;
                test_sfixed64_ = GET_RANDOM_INT;
                test_uint64_ = GET_RANDOM_INT;
                test_fixed64_ = GET_RANDOM_INT;
                test_bool_ = true;
                test_bytes_ = "bytes";
            }
#endif
        };
    }  // namespace protobuf
}  // namespace ariles_tests
