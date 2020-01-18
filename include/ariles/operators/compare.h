/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <limits>

namespace ariles
{
    namespace compare
    {
        class ARILES_VISIBILITY_ATTRIBUTE Iterator
        {
            public:
                class CompareParameters
                {
                public:
                    double float_tolerance_;
                    double double_tolerance_;
                    bool compare_number_of_entries_;
                    bool throw_on_error_;
                    bool nan_equal_;
                    bool inf_equal_;


                public:
                    CompareParameters()
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


                    template <typename t_Scalar>
                    t_Scalar getTolerance() const;
                };

                static const CompareParameters default_parameters_;



            public:
                template<class t_Configurable, class t_Other>
                    void start( const t_Configurable & configurable,
                                const t_Other & other,
                                const CompareParameters & parameters) const
                {
                    ARILES_TRACE_FUNCTION;

                    if (true == parameters.compare_number_of_entries_)
                    {
                        if (configurable.getNumberOfEntries() != other.getNumberOfEntries())
                        {
                            ARILES_THROW("Comparison failed: dfferent number of entries.");
                        }
                    }
                }


                template<class t_Configurable, class t_Other>
                    void finish(const t_Configurable &,
                                const t_Other &,
                                const CompareParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }


                template <typename t_Scalar>
                    static bool ARILES_VISIBILITY_ATTRIBUTE compareFloats(
                            const t_Scalar left,
                            const t_Scalar right,
                            const CompareParameters & param)
                {
                    if (isNaN(left))
                    {
                        if (isNaN(right))
                        {
                            return (param.nan_equal_);
                        }
                        else
                        {
                            return (false);
                        }
                    }

                    if (isInfinity(left))
                    {
                        if (isInfinity(right))
                        {
                            if (((left > 0) && (right > 0)) || ((left < 0) && (right < 0)))
                            {
                                return (param.inf_equal_);
                            }
                        }
                        return (false);
                    }
                    return (std::abs(left - right) <=
                            ( (std::abs(left) < std::abs(right) ? std::abs(right) : std::abs(left)) * param.double_tolerance_));
                }
        };


        template <>
        inline double Iterator::CompareParameters::getTolerance<double>() const
        {
            return (double_tolerance_);
        }

        template <>
        inline float Iterator::CompareParameters::getTolerance<float>() const
        {
            return (float_tolerance_);
        }



        class Base
        {
            public:
        };

    }  // namespace compare

    /// @todo DEPRECATED
    typedef compare::Iterator::CompareParameters ComparisonParameters;
}  // namespace ariles
