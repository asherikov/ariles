/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "complex_base.h"

namespace ariles_tests
{
    class ConfigurableComplex : public ARILES_TEST_DEFAULT_BASE, public ConfigurableComplexBase<ConfigurableComplex>
    {
#define ARILES2_ENTRIES_STANDARD_TYPES(v)                                                                              \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, unsigned_integer, std::size_t)                                                             \
    ARILES2_TYPED_ENTRY_(v, real, double)                                                                              \
    ARILES2_TYPED_ENTRY_(v, complex_float, std::complex<float>)                                                        \
    ARILES2_TYPED_ENTRY_(v, string, std::string)                                                                       \
    ARILES2_TYPED_ENTRY_(v, std_vector, std::vector<double>)                                                           \
    ARILES2_TYPED_ENTRY_(v, std_nested_vector, std::vector<std::vector<double> >)                                      \
    ARILES2_TYPED_ENTRY_(v, some_enum, SomeEnum)                                                                       \
    ARILES2_TYPED_ENTRY_(v, boolean_true, bool)                                                                        \
    ARILES2_TYPED_ENTRY_(v, boolean_false, bool)                                                                       \
    ARILES2_ENTRY_(v, std_pair)                                                                                        \
    ARILES2_ENTRY_(v, std_map)


#ifdef ARILES_ADAPTER_EIGEN
#    define ARILES2_ENTRIES_0(v)                                                                                       \
        ARILES2_ENTRIES_STANDARD_TYPES(v)                                                                              \
        ARILES2_TYPED_ENTRY_(v, vector, Eigen::Vector3d)                                                               \
        ARILES2_TYPED_ENTRY_(v, matrix, Eigen::Matrix3d)                                                               \
        ARILES2_TYPED_ENTRY_(v, matrix_x, Eigen::MatrixXd)                                                             \
        ARILES2_TYPED_ENTRY_(v, matrix_complex_double, Eigen::Matrix3cd)                                               \
        ARILES2_TYPED_ENTRY_(v, std_vector_evector, std::vector<Eigen::Vector3d>)                                      \
        ARILES2_TYPED_ENTRY_(v, std_nested_vector_evector, std::vector<std::vector<Eigen::Vector3d> >)                 \
        ARILES2_TYPED_ENTRY_(v, isometry, Eigen::Isometry3d)                                                           \
        ARILES2_TYPED_ENTRY_(v, quaternion, Eigen::Quaterniond)
#else
#    define ARILES2_ENTRIES_0(v) ARILES2_ENTRIES_STANDARD_TYPES(v)
#endif

#ifdef ARILES_ADAPTER_BETTER_ENUMS
#    define ARILES2_ENTRIES_1(v)                                                                                       \
        ARILES2_ENTRIES_0(v)                                                                                           \
        ARILES2_TYPED_ENTRY_(v, better_enum, BetterEnum)
#else
#    define ARILES2_ENTRIES_1(v) ARILES2_ENTRIES_STANDARD_TYPES(v)
#endif

#define ARILES2_ENTRIES(v) ARILES2_ENTRIES_1(v)
#include ARILES2_INITIALIZE

#undef ARILES2_ENTRIES_STANDARD_TYPES
#undef ARILES2_ENTRIES_0
#undef ARILES2_ENTRIES_1


    public:
        std::pair<std::string, double> std_pair_;
        std::map<std::string, std::vector<std::string> > std_map_;


    public:
        ConfigurableComplex()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        virtual ~ConfigurableComplex()
        {
        }

        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            ConfigurableComplexBase<ConfigurableComplex>::arilesVisit(visitor, param);
        }
    };
}  // namespace ariles_tests
