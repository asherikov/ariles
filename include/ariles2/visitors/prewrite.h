/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"
#include "process.h"

/**
@defgroup prewrite PreWrite
@ingroup process

@brief Preprocess entries, e.g., pack values before serialization.
*/

namespace ariles2
{
    /// @ingroup prewrite
    namespace prewrite
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public visitor::Parameters
        {
        public:
            Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public ariles2::process::Visitor<const prewrite::Visitor, prewrite::Parameters>
        {
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::Base<const prewrite::Visitor>
        {
        };


#define ARILES2_NAMED_ENTRY_prewrite(v, entry, name) visitor.visitMapEntry(entry, #name, parameters);
#define ARILES2_PARENT_prewrite(v, entry)
#define ARILES2_VISIT_prewrite                                                                                         \
    template <class t_Visitor>                                                                                         \
    void arilesVisit(                                                                                                  \
            const t_Visitor &visitor,                                                                                  \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::prewrite::Visitor, t_Visitor))                                            \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        arilesVisitParents(visitor, parameters);                                                                       \
        ARILES2_ENTRIES(prewrite)                                                                                      \
    }

#define ARILES2_METHODS_prewrite ARILES2_METHODS(prewrite, const, ARILES2_EMPTY_MACRO)
#define ARILES2_BASE_METHODS_prewrite ARILES2_BASE_METHODS(prewrite)
    }  // namespace prewrite


    /// @ingroup prewrite
    using PreWrite = prewrite::Visitor;
}  // namespace ariles2
