/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "enum.h"

class ConfigurableComplexVerbose : public ariles::ConfigurableBase
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
        ARILES_ENTRY_(enum)
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


    public:
        ConfigurableComplexVerbose()
        {
            setDefaults();
        }


        virtual void setDefaults()
        {
            integer_ = 10;
            unsigned_integer_ = 100;
            real_ = 1.33;
            vector_.setConstant(3);
            matrix_ << 1, 2, 3, 4, 5, 6, 7, 8, 9;
            string_ = "blahblah";

            matrix_x_.resize(2, 3);
            matrix_x_ << 8, 7, 6, 3, 2, 1;

            std_vector_.resize(5);
            for(std::size_t i = 0; i < std_vector_.size(); ++i)
            {
                std_vector_[i] = i * 5.22 + 2.3;
            }

            std_nested_vector_.resize(3);
            for(std::size_t i = 0; i < std_nested_vector_.size(); ++i)
            {
                std_nested_vector_[i].resize(7-i);

                for(std::size_t j = 0; j < std_nested_vector_[i].size(); ++j)
                {
                    std_nested_vector_[i][j] = 5.2 + i*0.1 + j*0.3;
                }
            }

            std_vector_evector_.resize(4);
            for(std::size_t i = 0; i < std_vector_evector_.size(); ++i)
            {
                std_vector_evector_[i].setConstant(i*9 + 0.43);
            }

            std_nested_vector_evector_.resize(2);
            for(std::size_t i = 0; i < std_nested_vector_evector_.size(); ++i)
            {
                std_nested_vector_evector_[i].resize(1+i);

                for(std::size_t j = 0; j < std_nested_vector_evector_[i].size(); ++j)
                {
                    std_nested_vector_evector_[i][j].setConstant(5.2 + i*0.1 + j*0.3);
                }
            }

            enum_ = ANOTHER_VALUE;
        }


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            unsigned_integer_ = GET_RANDOM_UINT;
            real_ = GET_RANDOM_REAL;
            vector_.setRandom();
            matrix_.setRandom();
            string_ = "blahblah";

            matrix_x_.resize(2, 3);
            matrix_x_.setRandom();

            std_vector_.resize(5);
            for(std::size_t i = 0; i < std_vector_.size(); ++i)
            {
                std_vector_[i] = GET_RANDOM_REAL;
            }

            std_nested_vector_.resize(3);
            for(std::size_t i = 0; i < std_nested_vector_.size(); ++i)
            {
                std_nested_vector_[i].resize(7-i);

                for(std::size_t j = 0; j < std_nested_vector_[i].size(); ++j)
                {
                    std_nested_vector_[i][j] = GET_RANDOM_REAL;
                }
            }

            std_vector_evector_.resize(4);
            for(std::size_t i = 0; i < std_vector_evector_.size(); ++i)
            {
                std_vector_evector_[i].setRandom();
            }

            std_nested_vector_evector_.resize(2);
            for(std::size_t i = 0; i < std_nested_vector_evector_.size(); ++i)
            {
                std_nested_vector_evector_[i].resize(1+i);

                for(std::size_t j = 0; j < std_nested_vector_evector_[i].size(); ++j)
                {
                    std_nested_vector_evector_[i][j].setRandom();
                }
            }

            enum_ = ANOTHER_VALUE;
            finalize();
        }
};
