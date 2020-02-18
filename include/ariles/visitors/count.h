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
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::Visitor
        {
            public:
                class Parameters
                {
                };


            public:
                std::size_t counter_;
                bool descend_;


            public:
                Visitor()
                {
                    counter_ = 0;
                    descend_ = false;
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
                    counter_ = 0;
                    descend_ = true;
                }


                template<class t_Configurable>
                    void finishRoot(const t_Configurable &,
                                    const Parameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Entry>
                    void operator()(
                            const t_Entry & entry,
                            const std::string & name,
                            const Parameters & /*param*/,
                            ARILES_IS_BASE_DISABLER(visitor::ConstBase<count::Visitor>, t_Entry))
                {
                    ARILES_UNUSED_ARG(name);
                    ARILES_UNUSED_ARG(entry);
                    ARILES_TRACE_FUNCTION;
                    ARILES_TRACE_ENTRY(name);
                    ARILES_TRACE_TYPE(entry);
                    ++this->counter_;
                }


                template<class t_Entry>
                    void operator()(
                            const t_Entry & entry,
                            const std::string & name,
                            const Parameters & param,
                            ARILES_IS_BASE_ENABLER(visitor::ConstBase<count::Visitor>, t_Entry))
                {
                    ARILES_UNUSED_ARG(name);
                    ARILES_TRACE_FUNCTION;
                    ARILES_TRACE_ENTRY(name);
                    ARILES_TRACE_TYPE(entry);
                    if (true == this->descend_)
                    {
                        this->descend_ = false;
                        entry.arilesVirtualVisit(*this, param);
                    }
                    else
                    {
                        ++this->counter_;
                    }
                }
        };



        class ARILES_VISIBILITY_ATTRIBUTE Base
            : public visitor::ConstBase<count::Visitor>
        {
            public:
        };

#ifndef ARILES_METHODS_count
#   define ARILES_METHODS_count ARILES_METHODS(ariles::count::Visitor, const)
#endif
    }
}
