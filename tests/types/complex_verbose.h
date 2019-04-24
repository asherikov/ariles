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
    class ConfigurableComplexVerbose : public ariles::ConfigurableBase, public ConfigurableComplexBase<ConfigurableComplexVerbose>
    {
        #define ARILES_SECTION_ID "ConfigurableComplexVerbose"
        #define ARILES_CONSTRUCTOR ConfigurableComplexVerbose
        #define ARILES_ENTRIES_STANDARD_TYPES \
            ARILES_ENTRY_(integer) \
            ARILES_ENTRY_(unsigned_integer) \
            ARILES_ENTRY_(real) \
            ARILES_ENTRY_(string) \
            ARILES_ENTRY_(std_vector) \
            ARILES_ENTRY_(std_nested_vector) \
            ARILES_ENTRY_(enum) \
            ARILES_ENTRY_(boolean_true) \
            ARILES_ENTRY_(boolean_false) \
            ARILES_ENTRY_(std_pair) \
            ARILES_ENTRY_(std_map)

#ifdef ARILES_ADAPTER_EIGEN
        #define ARILES_ENTRIES_0 \
            ARILES_ENTRIES_STANDARD_TYPES \
            ARILES_ENTRY_(vector) \
            ARILES_ENTRY_(matrix) \
            ARILES_ENTRY_(matrix_x) \
            ARILES_ENTRY_(std_vector_evector) \
            ARILES_ENTRY_(std_nested_vector_evector) \
            ARILES_ENTRY_(isometry) \
            ARILES_ENTRY_(quaternion)
#else
        #define ARILES_ENTRIES_0 ARILES_ENTRIES_STANDARD_TYPES
#endif

#ifdef ARILES_ADAPTER_BETTER_ENUMS
        #define ARILES_ENTRIES_1 \
            ARILES_ENTRIES_0 \
            ARILES_ENTRY_(better_enum) 
#else
        #define ARILES_ENTRIES_1 ARILES_ENTRIES_STANDARD_TYPES
#endif


        #define ARILES_ENTRIES ARILES_ENTRIES_1
        #include ARILES_INITIALIZE

#undef ARILES_ENTRIES_STANDARD_TYPES
#undef ARILES_ENTRIES_0
#undef ARILES_ENTRIES_1


        public:
            int                     integer_;
            std::size_t             unsigned_integer_; \
            double                  real_;


            std::vector<double>                 std_vector_;
            std::vector< std::vector<double> >  std_nested_vector_;

            std::string             string_;

            bool boolean_true_;
            bool boolean_false_;

            SomeEnum enum_;
            BetterEnum better_enum_;

            std::pair<std::string, double> std_pair_;
            std::map<std::string, std::vector<std::string> > std_map_;


#ifdef ARILES_ADAPTER_EIGEN
            Eigen::Vector3d         vector_;
            Eigen::Matrix3d         matrix_;
            Eigen::MatrixXd         matrix_x_;

            std::vector<Eigen::Vector3d>                std_vector_evector_;
            std::vector< std::vector<Eigen::Vector3d> > std_nested_vector_evector_;

            Eigen::Isometry3d   isometry_;

            Eigen::Quaterniond  quaternion_;
#endif


        public:
            ConfigurableComplexVerbose()
            {
                setDefaults();
            }

            virtual ~ConfigurableComplexVerbose() {}

            void setDefaults()
            {
                ConfigurableComplexBase<ConfigurableComplexVerbose>::setDefaults();
            }
    };
}
