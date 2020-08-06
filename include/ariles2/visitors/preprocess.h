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
@defgroup preprocess PreProcess
@ingroup process

@brief Preprocess entries, e.g., pack values before serialization.
*/

namespace ariles2
{
    /// @ingroup preprocess
    namespace preprocess
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public visitor::Parameters
        {
        public:
            Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public ariles2::process::Visitor<const preprocess::Visitor, preprocess::Parameters>
        {
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::Base<const preprocess::Visitor>
        {
        };


#define ARILES2_NAMED_ENTRY_preprocess(v, entry, name) visitor.visitMapEntry(entry, #name, parameters);
#define ARILES2_PARENT_preprocess(v, entry)
#define ARILES2_VISIT_preprocess                                                                                       \
    template <class t_Visitor>                                                                                         \
    void arilesVisit(                                                                                                  \
            const t_Visitor &visitor,                                                                                  \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::preprocess::Visitor, t_Visitor))                                          \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        arilesVisitParents(visitor, parameters);                                                                       \
        ARILES2_ENTRIES(preprocess)                                                                                    \
    }

#define ARILES2_METHODS_preprocess ARILES2_METHODS(preprocess, const, ARILES2_EMPTY_MACRO)
#define ARILES2_BASE_METHODS_preprocess ARILES2_BASE_METHODS(preprocess)
    }  // namespace preprocess


    /// @ingroup preprocess
    typedef preprocess::Visitor PreProcess;
}  // namespace ariles2
