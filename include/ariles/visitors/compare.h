/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

namespace ariles
{
    namespace compare
    {
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::Visitor
        {
            public:
                class Parameters
                {
                public:
                    double float_tolerance_;
                    double double_tolerance_;
                    bool compare_number_of_entries_;
                    bool throw_on_error_; /// @todo DEPRECATED
                    bool nan_equal_;
                    bool inf_equal_;
                    /// @todo continue on failure.


                public:
                    Parameters()
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

            public:
                bool equal_;



            public:
                template<class t_Left, class t_Right>
                    bool compare(const t_Left &left, const t_Right &right, const Parameters &param)
                {
                    ariles::apply(*this, left, right, param);
                    return (equal_);
                }

                const Parameters & getDefaultParameters() const
                {
                    const static Parameters parameters;
                    return parameters;
                }


                template<class t_Configurable>
                    void startRoot( const t_Configurable &,
                                    const Parameters &)
                {
                    ARILES_TRACE_FUNCTION;
                    equal_ = true;
                }


                template<class t_Configurable>
                    void finishRoot(const t_Configurable &,
                                    const Parameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }


                template <typename t_Scalar>
                    static bool ARILES_VISIBILITY_ATTRIBUTE compareFloats(
                            const t_Scalar left,
                            const t_Scalar right,
                            const Parameters & param)
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
        inline double Visitor::Parameters::getTolerance<double>() const
        {
            return (double_tolerance_);
        }

        template <>
        inline float Visitor::Parameters::getTolerance<float>() const
        {
            return (float_tolerance_);
        }


#ifndef ARILES_METHODS_compare
#   define ARILES_METHODS_compare \
        const ariles::compare::Visitor::Parameters & \
            arilesGetParameters(const ariles::compare::Visitor &visitor) const \
        { \
            ARILES_TRACE_FUNCTION; \
            return(visitor.getDefaultParameters()); \
        }
#endif
    }  // namespace compare
}  // namespace ariles
