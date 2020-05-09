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
    class ConfigurableComplex : public ariles::DefaultBase, public ConfigurableComplexBase<ConfigurableComplex>
    {
#define ARILES_ENTRIES_STANDARD_TYPES(v)                                                                               \
    ARILES_TYPED_ENTRY_(v, integer, int)                                                                               \
    ARILES_TYPED_ENTRY_(v, unsigned_integer, std::size_t)                                                              \
    ARILES_TYPED_ENTRY_(v, real, double)                                                                               \
    ARILES_TYPED_ENTRY_(v, string, std::string)                                                                        \
    ARILES_TYPED_ENTRY_(v, std_vector, std::vector<double>)                                                            \
    ARILES_TYPED_ENTRY_(v, std_nested_vector, std::vector<std::vector<double> >)                                       \
    ARILES_TYPED_ENTRY_(v, enum, SomeEnum)                                                                             \
    ARILES_TYPED_ENTRY_(v, boolean_true, bool)                                                                         \
    ARILES_TYPED_ENTRY_(v, boolean_false, bool)                                                                        \
    ARILES_ENTRY_(v, std_pair)                                                                                         \
    ARILES_ENTRY_(v, std_map)


#ifdef ARILES_ADAPTER_EIGEN
#    define ARILES_ENTRIES_0(v)                                                                                        \
        ARILES_ENTRIES_STANDARD_TYPES(v)                                                                               \
        ARILES_TYPED_ENTRY_(v, vector, Eigen::Vector3d)                                                                \
        ARILES_TYPED_ENTRY_(v, matrix, Eigen::Matrix3d)                                                                \
        ARILES_TYPED_ENTRY_(v, matrix_x, Eigen::MatrixXd)                                                              \
        ARILES_TYPED_ENTRY_(v, std_vector_evector, std::vector<Eigen::Vector3d>)                                       \
        ARILES_TYPED_ENTRY_(v, std_nested_vector_evector, std::vector<std::vector<Eigen::Vector3d> >)                  \
        ARILES_TYPED_ENTRY_(v, isometry, Eigen::Isometry3d)                                                            \
        ARILES_TYPED_ENTRY_(v, quaternion, Eigen::Quaterniond)
#else
#    define ARILES_ENTRIES_0(v) ARILES_ENTRIES_STANDARD_TYPES(v)
#endif

#ifdef ARILES_ADAPTER_BETTER_ENUMS
#    define ARILES_ENTRIES_1(v)                                                                                        \
        ARILES_ENTRIES_0(v)                                                                                            \
        ARILES_TYPED_ENTRY_(v, better_enum, BetterEnum)
#else
#    define ARILES_ENTRIES_1(v) ARILES_ENTRIES_STANDARD_TYPES(v)
#endif

#define ARILES_ENTRIES(v) ARILES_ENTRIES_1(v)
#include ARILES_INITIALIZE

#undef ARILES_ENTRIES_STANDARD_TYPES
#undef ARILES_ENTRIES_0
#undef ARILES_ENTRIES_1


    public:
        std::pair<std::string, double> std_pair_;
        std::map<std::string, std::vector<std::string> > std_map_;


    public:
        ConfigurableComplex()
        {
            ariles::apply<ariles::Defaults>(*this);
        }

        virtual ~ConfigurableComplex()
        {
        }

        void arilesVisit(const ariles::Defaults &visitor, const ariles::Defaults::Parameters &param)
        {
            ConfigurableComplexBase<ConfigurableComplex>::arilesVisit(visitor, param);
        }
    };
}  // namespace ariles_tests
