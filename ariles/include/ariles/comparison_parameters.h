/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    class ComparisonParameters
    {
        public:
            double float_tolerance_;
            double double_tolerance_;
            bool compare_number_of_entries_;
            bool throw_on_error_;
            bool nan_equal_;
            bool inf_equal_;


        public:
            ComparisonParameters()
            {
                setDefaults();
            }


            void setDefaults()
            {
                double_tolerance_ = 1e-12;
                float_tolerance_ = 1e-8;
                compare_number_of_entries_ = false;
                throw_on_error_ = false;

                nan_equal_ = true;
                inf_equal_ = true;
            }


            template<typename t_Scalar>
                t_Scalar getTolerance() const;
    };


    template<> inline double ComparisonParameters::getTolerance<double>() const
    {
        return (double_tolerance_);
    }

    template<> inline float ComparisonParameters::getTolerance<float>() const
    {
        return (float_tolerance_);
    }
}
