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
    class ConfigurableComplexVerbose : public ARILES_TEST_DEFAULT_BASE,
                                       public ConfigurableComplexBase<ConfigurableComplexVerbose>
    {
#define ARILES2_ENTRIES_STANDARD_TYPES(v)                                                                              \
    ARILES2_ENTRY_(v, integer)                                                                                         \
    ARILES2_ENTRY_(v, unsigned_integer)                                                                                \
    ARILES2_ENTRY_(v, real)                                                                                            \
    ARILES2_ENTRY_(v, string)                                                                                          \
    ARILES2_ENTRY_(v, std_vector)                                                                                      \
    ARILES2_ENTRY_(v, std_nested_vector)                                                                               \
    ARILES2_ENTRY_(v, enum)                                                                                            \
    ARILES2_ENTRY_(v, boolean_true)                                                                                    \
    ARILES2_ENTRY_(v, boolean_false)                                                                                   \
    ARILES2_ENTRY_(v, std_pair)                                                                                        \
    ARILES2_ENTRY_(v, std_map)

#ifdef ARILES_ADAPTER_EIGEN
#    define ARILES2_ENTRIES_0(v)                                                                                       \
        ARILES2_ENTRIES_STANDARD_TYPES(v)                                                                              \
        ARILES2_ENTRY_(v, vector)                                                                                      \
        ARILES2_ENTRY_(v, matrix)                                                                                      \
        ARILES2_ENTRY_(v, matrix_x)                                                                                    \
        ARILES2_ENTRY_(v, std_vector_evector)                                                                          \
        ARILES2_ENTRY_(v, std_nested_vector_evector)                                                                   \
        ARILES2_ENTRY_(v, isometry)                                                                                    \
        ARILES2_ENTRY_(v, quaternion)
#else
#    define ARILES2_ENTRIES_0(v) ARILES2_ENTRIES_STANDARD_TYPES(v)
#endif

#ifdef ARILES_ADAPTER_BETTER_ENUMS
#    define ARILES2_ENTRIES_1(v)                                                                                       \
        ARILES2_ENTRIES_0(v)                                                                                           \
        ARILES2_ENTRY_(v, better_enum)
#else
#    define ARILES2_ENTRIES_1(v) ARILES2_ENTRIES_STANDARD_TYPES(v)
#endif


#define ARILES2_ENTRIES(v) ARILES2_ENTRIES_1(v)
#include ARILES2_INITIALIZE

#undef ARILES2_ENTRIES_STANDARD_TYPES
#undef ARILES2_ENTRIES_0
#undef ARILES2_ENTRIES_1


    public:
        int integer_;
        std::size_t unsigned_integer_;
        double real_;


        std::vector<double> std_vector_;
        std::vector<std::vector<double> > std_nested_vector_;

        std::string string_;

        bool boolean_true_;
        bool boolean_false_;

        SomeEnum enum_;
        BetterEnum better_enum_;

        std::pair<std::string, double> std_pair_;
        std::map<std::string, std::vector<std::string> > std_map_;


#ifdef ARILES_ADAPTER_EIGEN
        Eigen::Vector3d vector_;
        Eigen::Matrix3d matrix_;
        Eigen::MatrixXd matrix_x_;

        std::vector<Eigen::Vector3d> std_vector_evector_;
        std::vector<std::vector<Eigen::Vector3d> > std_nested_vector_evector_;

        Eigen::Isometry3d isometry_;

        Eigen::Quaterniond quaternion_;
#endif


    public:
        ConfigurableComplexVerbose()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        virtual ~ConfigurableComplexVerbose()
        {
        }

        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            ConfigurableComplexBase<ConfigurableComplexVerbose>::arilesVisit(visitor, param);
        }
    };
}  // namespace ariles_tests
