/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "count.h"

/**
@defgroup count_missing Count missing entries

@brief Counts number of missing entries.
*/

namespace ariles2
{
    /// @ingroup count_missing
    namespace count_missing
    {
        using Parameters = ARILES2_VISIBILITY_ATTRIBUTE visitor::Parameters;


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public ariles2::visitor::Base<Visitor, count::Parameters, std::size_t>
        {
        public:
            using Parameters = count::Parameters;


        public:
            template <class t_Entry>
            std::size_t visit(const t_Entry &entry, const std::string &, const Parameters &param) const
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_TYPE(entry);
                return (entry.arilesVirtualVisit(*this, param));
            }
        };


        using Base = ARILES2_VISIBILITY_ATTRIBUTE entry::ConstBase<const Visitor, std::size_t>;


#define ARILES2_NAMED_ENTRY_count_missing(v, entry, name) +ariles2::isMissing(entry)
#define ARILES2_PARENT_count_missing(v, entry) +entry::arilesVisit(visitor, parameters)

#define ARILES2_VISIT_count_missing                                                                                    \
    template <class t_Visitor>                                                                                         \
    std::size_t arilesVisit(                                                                                           \
            const t_Visitor &visitor,                                                                                  \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::count_missing::Visitor, t_Visitor)) const                                 \
    {                                                                                                                  \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        return (0 ARILES2_ENTRIES(count_missing));                                                                     \
    }

#define ARILES2_METHODS_count_missing                                                                                  \
    std::size_t arilesVirtualVisit(                                                                                    \
            const ariles2::count_missing::Visitor &visitor, const ariles2::count_missing::Visitor::Parameters &param)  \
            const override                                                                                             \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        return (this->arilesVisit(visitor, param));                                                                    \
    }                                                                                                                  \
    using ariles2::count_missing::Base::arilesGetParameters;

#define ARILES2_BASE_METHODS_count_missing ARILES2_BASE_METHODS(count_missing)
    }  // namespace count_missing


    /// @ingroup count_missing
    using CountMissing = count_missing::Visitor;
}  // namespace ariles2
