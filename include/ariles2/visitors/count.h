/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

namespace ariles2
{
    namespace count
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public visitor::Parameters
        {
        public:
            Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public ariles2::visitor::Base<visitor::Visitor, count::Parameters>
        {
        public:
            typedef count::Parameters Parameters;


        public:
            using visitor::Base<visitor::Visitor, Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }


            template <class t_Entry>
            std::size_t count(const t_Entry &entry, const Parameters &param) const
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_TYPE(entry);
                return (entry.arilesVirtualVisit(*this, param));
            }


            template <class t_Entry>
            std::size_t count(const t_Entry &entry) const
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_TYPE(entry);
                return (entry.arilesVirtualVisit(*this, entry.arilesGetParameters(*this)));
            }
        };



        class ARILES2_VISIBILITY_ATTRIBUTE Base
        {
        public:
            virtual std::size_t arilesVirtualVisit(const Visitor &, const Visitor::Parameters &) const = 0;

            virtual const Visitor::Parameters &arilesGetParameters(const Visitor &visitor) const
            {
                ARILES2_TRACE_FUNCTION;
                return (visitor.getDefaultParameters());
            }
        };



#define ARILES2_NAMED_ENTRY_count(v, entry, name) +1
#define ARILES2_PARENT_count(v, entry) +entry::arilesVisit(visitor, parameters)

#define ARILES2_VISIT_count                                                                                            \
    template <class t_Visitor>                                                                                         \
    std::size_t arilesVisit(                                                                                           \
            const t_Visitor &visitor,                                                                                  \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::count::Visitor, t_Visitor)) const                                         \
    {                                                                                                                  \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        return (0 ARILES2_ENTRIES(count));                                                                             \
    }

#define ARILES2_METHODS_count                                                                                          \
    virtual std::size_t arilesVirtualVisit(                                                                            \
            const ariles2::count::Visitor &visitor, const ariles2::count::Visitor::Parameters &param) const            \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        return (this->arilesVisit(visitor, param));                                                                    \
    }                                                                                                                  \
    using ariles2::count::Base::arilesGetParameters;

#define ARILES2_BASE_METHODS_count ARILES2_BASE_METHODS(count)
    }  // namespace count


    typedef count::Visitor Count;
}  // namespace ariles2
