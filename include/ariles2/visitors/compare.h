/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/math/special_functions.hpp>
#include "common.h"

/**
@defgroup compare Compare

@brief Class comparison.
*/

namespace ariles2
{
    /// @ingroup compare
    namespace compare
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public visitor::Parameters
        {
        public:
            double float_tolerance_;
            double double_tolerance_;
            bool compare_number_of_entries_;
            bool nan_equal_;
            bool inf_equal_;
            /// @todo continue on failure.


        public:
            Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
                setDefaults();
            }


            void setDefaults()
            {
                double_tolerance_ = 1e-12;
                float_tolerance_ = 1e-8;
                compare_number_of_entries_ = false;

                nan_equal_ = true;
                inf_equal_ = true;
            }


            template <typename t_Scalar>
            t_Scalar getTolerance(const typename ARILES2_IS_FLOATING_POINT_ENABLER_TYPE(t_Scalar) = NULL) const;

            template <class t_Complex>
            typename t_Complex::value_type getTolerance() const
            {
                return (getTolerance<typename t_Complex::value_type>());
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public visitor::Base<visitor::Visitor, compare::Parameters, bool>
        {
        public:
            typedef compare::Parameters Parameters;


        public:
            bool equal_;
            std::vector<std::string> backtrace_;


        public:
            using visitor::Base<visitor::Visitor, Parameters, bool>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }


            template <class t_Left, class t_Right>
            bool visit(const t_Left &left, const t_Right &right, const std::string &name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                try
                {
                    equal_ = true;
                    this->visitMapEntry(left, right, name, param);
                    if (false == equal_)
                    {
                        backtrace_.push_back(name);
                    }
                }
                catch (std::exception &e)
                {
                    backtrace_.push_back(e.what());
                    equal_ = false;
                }
                return (equal_);
            }


            template <typename t_Scalar>
            static bool compareFloats(const t_Scalar left, const t_Scalar right, const Parameters &param)
            {
                if (boost::math::isnan(left))
                {
                    if (boost::math::isnan(right))
                    {
                        return (param.nan_equal_);
                    }
                    else
                    {
                        return (false);
                    }
                }

                if (boost::math::isinf(left))
                {
                    if (boost::math::isinf(right))
                    {
                        if (((left > 0) && (right > 0)) || ((left < 0) && (right < 0)))
                        {
                            return (param.inf_equal_);
                        }
                    }
                    return (false);
                }
                return (std::abs(left - right)
                        <= ((std::abs(left) < std::abs(right) ? std::abs(right) : std::abs(left))
                            * param.double_tolerance_));
            }


            template <class t_Left, class t_Right>
            void visitMapEntry(
                    const t_Left &left,
                    const t_Right &right,
                    const std::string &name,
                    const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);
                ARILES2_TRACE_TYPE(left);
                ARILES2_TRACE_TYPE(right);

                const bool equal_check = this->equal_;
                apply_compare(*this, left, right, param);
                if (false == this->equal_ and equal_check != this->equal_)
                {
                    backtrace_.push_back(name);
                }
            }
        };


        template <>
        inline double Visitor::Parameters::getTolerance<double>(
                const ARILES2_IS_FLOATING_POINT_ENABLER_TYPE(double)) const
        {
            return (double_tolerance_);
        }

        template <>
        inline float Visitor::Parameters::getTolerance<float>(const ARILES2_IS_FLOATING_POINT_ENABLER_TYPE(float)) const
        {
            return (float_tolerance_);
        }


        class ARILES2_VISIBILITY_ATTRIBUTE Base
        {
        };


#define ARILES2_NAMED_ENTRY_compare(v, entry, name) visitor.visitMapEntry(entry, other.entry, #name, parameters);
#define ARILES2_PARENT_compare(v, entry) entry::arilesVisit(visitor, other, parameters);

#define ARILES2_VISIT_compare                                                                                          \
    template <class t_Other>                                                                                           \
    void arilesVisit(                                                                                                  \
            ariles2::compare::Visitor &visitor,                                                                        \
            const t_Other &other,                                                                                      \
            const typename ariles2::compare::Visitor::Parameters &parameters) const                                    \
    {                                                                                                                  \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(other);                                                                                     \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_ENTRIES(compare)                                                                                       \
    }

#define ARILES2_METHODS_compare                                                                                        \
    const ariles2::compare::Visitor::Parameters &arilesGetParameters(const ariles2::compare::Visitor &visitor) const   \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        return (visitor.getDefaultParameters());                                                                       \
    }
#define ARILES2_BASE_METHODS_compare
    }  // namespace compare


    /// @ingroup compare
    typedef compare::Visitor Compare;
}  // namespace ariles2
