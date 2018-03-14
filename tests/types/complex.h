/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "enum.h"

class ConfigurableComplex : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableComplex"
    #define ARILES_CONSTRUCTOR ConfigurableComplex
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer,     int) \
        ARILES_TYPED_ENTRY_(unsigned_integer, std::size_t) \
        ARILES_TYPED_ENTRY_(real,        double) \
        ARILES_TYPED_ENTRY_(string,      std::string) \
        ARILES_TYPED_ENTRY_(vector,      Eigen::Vector3d) \
        ARILES_TYPED_ENTRY_(matrix,      Eigen::Matrix3d) \
        ARILES_TYPED_ENTRY_(matrix_x,    Eigen::MatrixXd) \
        ARILES_TYPED_ENTRY_(std_vector,          std::vector<double>) \
        ARILES_TYPED_ENTRY_(std_nested_vector,   std::vector< std::vector<double> >) \
        ARILES_TYPED_ENTRY_(std_vector_evector,  std::vector<Eigen::Vector3d>) \
        ARILES_TYPED_ENTRY_(std_nested_vector_evector, std::vector< std::vector<Eigen::Vector3d> >) \
        ARILES_TYPED_ENTRY_(enum, SomeEnum) \
        ARILES_ENTRY_(std_pair)
    #include ARILES_INITIALIZE

    public:
        std::pair<std::string, double>  std_pair_;


    public:
        ConfigurableComplex()
        {
            setDefaults();
        }


        virtual void setDefaults()
        {
            integer_ = 10;
            unsigned_integer_ = 100;
            real_ = 1.33;
            string_ = "blahblah";
            vector_.setConstant(3);
            matrix_ << 1, 2, 3, 4, 5, 6, 7, 8, 9;

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

            std_pair_.first = "test";
            std_pair_.second = 13;
        }


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            unsigned_integer_ = GET_RANDOM_UINT;
            real_ = GET_RANDOM_REAL;
            string_ = "blahblah";
            vector_.setRandom();
            matrix_.setRandom();

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

            std_pair_.first = "testtt";
            std_pair_.second = GET_RANDOM_REAL;

            finalize();
        }
};
