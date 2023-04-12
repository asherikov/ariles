/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles_tests
{
    class CopyComplex
    {
    public:
        int integer;
        std::size_t unsigned_integer;
        double real;
        std::complex<float> complex_float;


        std::vector<double> std_vector;
        std::vector<std::vector<double>> std_nested_vector;

        std::string string;

        bool boolean_true;
        bool boolean_false;

        SomeEnum some_enum;
        BetterEnum better_enum;

        std::pair<std::string, double> std_pair;
        std::map<std::string, std::vector<std::string>> std_map;


#ifdef ARILES_ADAPTER_EIGEN
        Eigen::Vector3d vector;
        Eigen::Matrix3d matrix;
        Eigen::MatrixXd matrix_x;
        Eigen::Matrix3cd matrix_complex_double;

        std::vector<Eigen::Vector3d> std_vector_evector;
        std::vector<std::vector<Eigen::Vector3d>> std_nested_vector_evector;

        Eigen::Isometry3d isometry;

        Eigen::Quaterniond quaternion;
#endif
    };
}  // namespace ariles_tests
