/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "complex_base.h"

class ConfigurableComplexVerbose : public ariles::ConfigurableBase, public ConfigurableComplexBase<ConfigurableComplexVerbose>
{
    #define ARILES_SECTION_ID "ConfigurableComplexVerbose"
    #define ARILES_CONSTRUCTOR ConfigurableComplexVerbose
    #define ARILES_ENTRIES \
        ARILES_ENTRY_(integer) \
        ARILES_ENTRY_(unsigned_integer) \
        ARILES_ENTRY_(real) \
        ARILES_ENTRY_(vector) \
        ARILES_ENTRY_(matrix) \
        ARILES_ENTRY_(matrix_x) \
        ARILES_ENTRY_(std_vector) \
        ARILES_ENTRY_(std_nested_vector) \
        ARILES_ENTRY_(string) \
        ARILES_ENTRY_(std_vector_evector) \
        ARILES_ENTRY_(std_nested_vector_evector) \
        ARILES_ENTRY_(enum) \
        ARILES_ENTRY_(isometry) \
        ARILES_ENTRY_(std_pair) \
        ARILES_ENTRY_(std_map)
    #include ARILES_INITIALIZE


    public:
        int                     integer_;
        std::size_t             unsigned_integer_; \
        double                  real_;

        Eigen::Vector3d         vector_;
        Eigen::Matrix3d         matrix_;
        Eigen::MatrixXd         matrix_x_;

        std::vector<double>                 std_vector_;
        std::vector< std::vector<double> >  std_nested_vector_;

        std::string             string_;

        std::vector<Eigen::Vector3d>                std_vector_evector_;
        std::vector< std::vector<Eigen::Vector3d> > std_nested_vector_evector_;

        SomeEnum enum_;

        std::pair<std::string, double> std_pair_;
        std::map<std::string, std::vector<std::string> > std_map_;

        Eigen::Isometry3d   isometry_;


    public:
        ConfigurableComplexVerbose()
        {
            setDefaults();
        }

        virtual void setDefaults()
        {
            ConfigurableComplexBase<ConfigurableComplexVerbose>::setDefaults();
        }
};
