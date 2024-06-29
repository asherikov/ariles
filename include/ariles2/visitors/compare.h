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
            float float_tolerance_;
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


            template <typename t_Scalar, typename = std::enable_if_t<std::is_floating_point_v<t_Scalar>>>
            t_Scalar getTolerance() const
            {
                if constexpr (std::is_same_v<t_Scalar, double>)
                {
                    return (double_tolerance_);
                }
                if constexpr (std::is_same_v<t_Scalar, float>)
                {
                    return (float_tolerance_);
                }
            }


            template <class t_Complex>
            typename t_Complex::value_type getTolerance() const
            {
                return (getTolerance<typename t_Complex::value_type>());
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public visitor::Base<Visitor, compare::Parameters, bool>
        {
        public:
            using Parameters = compare::Parameters;


        public:
            bool equal_;
            std::vector<std::string> backtrace_;


        public:
            template <class t_Left, class t_Right>
            bool visit(const t_Left &left, const t_Right &right, const std::string &name, const Parameters &param)
            {
                CPPUT_TRACE_FUNCTION;
                try
                {
                    equal_ = true;
                    this->visitMapEntry(left, right, name, param);
                    if (not equal_)
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
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(name);
                CPPUT_TRACE_TYPE(left);
                CPPUT_TRACE_TYPE(right);

                const bool equal_check = this->equal_;
                apply_compare(*this, left, right, param);
                if (not this->equal_ and equal_check != this->equal_)
                {
                    backtrace_.push_back(name);
                }
            }
        };


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
        CPPUT_UNUSED_ARG(visitor);                                                                                     \
        CPPUT_UNUSED_ARG(other);                                                                                       \
        CPPUT_UNUSED_ARG(parameters);                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        ARILES2_ENTRIES(compare)                                                                                       \
    }

#define ARILES2_METHODS_compare                                                                                        \
    const ariles2::compare::Visitor::Parameters &arilesGetParameters(const ariles2::compare::Visitor &visitor) const   \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        return (visitor.getDefaultParameters());                                                                       \
    }
#define ARILES2_BASE_METHODS_compare
    }  // namespace compare


    /// @ingroup compare
    using Compare = compare::Visitor;
}  // namespace ariles2
