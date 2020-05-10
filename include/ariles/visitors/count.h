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
    namespace count
    {
        class ARILES_VISIBILITY_ATTRIBUTE Parameters
        {
        };


        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::Base<visitor::Visitor, count::Parameters>
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
            std::size_t operator()(const t_Entry &entry, const Parameters &param)
            {
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_TYPE(entry);
                return (entry.arilesVirtualVisit(*this, param));
            }


            template <class t_Entry>
            std::size_t operator()(const t_Entry &entry)
            {
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_TYPE(entry);
                return (entry.arilesVirtualVisit(*this, entry.arilesGetParameters(*this)));
            }
        };



        class ARILES_VISIBILITY_ATTRIBUTE Base
        {
        public:
            virtual std::size_t arilesVirtualVisit(Visitor &, const Visitor::Parameters &) const = 0;

            virtual const Visitor::Parameters &arilesGetParameters(const Visitor &visitor) const
            {
                ARILES_TRACE_FUNCTION;
                return (visitor.getDefaultParameters());
            }
        };

#define ARILES_NAMED_ENTRY_count(v, entry, name) +1
#define ARILES_PARENT_count(v, entry) +entry::arilesVisit(visitor, parameters)

#define ARILES_VISIT_count                                                                                             \
    std::size_t arilesVisit(ariles::count::Visitor &visitor, const ariles::count::Visitor::Parameters &parameters)     \
            const                                                                                                      \
    {                                                                                                                  \
        ARILES_UNUSED_ARG(visitor);                                                                                    \
        ARILES_UNUSED_ARG(parameters);                                                                                 \
        ARILES_TRACE_FUNCTION;                                                                                         \
        return (0 ARILES_ENTRIES(count));                                                                              \
    }

#define ARILES_METHODS_count                                                                                           \
    virtual std::size_t arilesVirtualVisit(                                                                            \
            ariles::count::Visitor &visitor, const ariles::count::Visitor::Parameters &param) const                    \
    {                                                                                                                  \
        ARILES_TRACE_FUNCTION;                                                                                         \
        return (this->arilesVisit(visitor, param));                                                                    \
    }                                                                                                                  \
    using ariles::count::Base::arilesGetParameters;
    }  // namespace count


    typedef count::Visitor Count;
}  // namespace ariles
