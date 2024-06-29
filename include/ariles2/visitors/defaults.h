/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <limits>
#include "common.h"

/**
@defgroup defaults Defaults (PreRead)

@brief Initializes entries to their default values.
*/

namespace ariles2
{
    /// @ingroup defaults
    namespace defaults
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public visitor::Parameters
        {
        public:
            double default_double_value_;
            float default_float_value_;

        public:
            Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
#ifdef ARILES2_DEFAULT_DOUBLE_VALUE
                default_double_value_ = ARILES2_DEFAULT_DOUBLE_VALUE;
#else
                default_double_value_ = std::numeric_limits<double>::signaling_NaN();
#endif

#ifdef ARILES2_DEFAULT_FLOAT_VALUE
                default_float_value_ = ARILES2_DEFAULT_FLOAT_VALUE;
#else
                default_float_value_ = std::numeric_limits<float>::signaling_NaN();
#endif
            }

            template <typename t_Scalar>
            inline t_Scalar getDefault() const
            {
                if constexpr (std::is_same_v<double, t_Scalar>)
                {
                    return default_double_value_;
                }
                if constexpr (std::is_same_v<float, t_Scalar>)
                {
                    return default_float_value_;
                }
                if constexpr (std::is_same_v<bool, t_Scalar>)
                {
                    return false;
                }
                if constexpr (std::is_same_v<std::string, t_Scalar>)
                {
                    return "";
                }
                return 0;
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public ariles2::visitor::Base<Visitor, defaults::Parameters>
        {
        public:
            using Parameters = defaults::Parameters;


        public:
            template <class t_Entry>
            void visit(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                CPPUT_TRACE_FUNCTION;
                this->visitMapEntry(entry, name, param);
            }

            template <class t_Entry>
            void visit(t_Entry &entry, const std::vector<std::string> &subtree, const Parameters &param) const
            {
                visit(entry, subtree.empty() ? "" : subtree.back(), param);
            }


            template <class t_Entry>
            void visitMapEntry(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                CPPUT_UNUSED_ARG(name);
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(name);
                CPPUT_TRACE_TYPE(entry);
                apply_defaults(*this, entry, param);
            }
        };


        using Base = entry::Base<const defaults::Visitor>;


#define ARILES2_NAMED_ENTRY_defaults(v, entry, name) visitor.visitMapEntry(entry, #name, parameters);
#define ARILES2_PARENT_defaults(v, entry)
#define ARILES2_VISIT_defaults                                                                                         \
    template <class t_Visitor>                                                                                         \
    void arilesVisit(                                                                                                  \
            const t_Visitor &visitor,                                                                                  \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::defaults::Visitor, t_Visitor))                                            \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        CPPUT_UNUSED_ARG(visitor);                                                                                     \
        CPPUT_UNUSED_ARG(parameters);                                                                                  \
        arilesVisitParents(visitor, parameters);                                                                       \
        ARILES2_ENTRIES(defaults)                                                                                      \
    }

#define ARILES2_METHODS_defaults ARILES2_METHODS(defaults, const, ARILES2_EMPTY_MACRO)
#define ARILES2_BASE_METHODS_defaults ARILES2_BASE_METHODS(defaults)
    }  // namespace defaults

    /**
     * @ingroup defaults
     * @{
     */
    using Defaults = defaults::Visitor;
    /// @}
}  // namespace ariles2
