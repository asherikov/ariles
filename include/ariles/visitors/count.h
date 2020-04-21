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
        class Parameters
        {
        };


        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::Base<count::Parameters>
        {
        public:
            typedef count::Parameters Parameters;


        public:
            std::size_t counter_;
            bool descend_;


        public:
            Visitor()
            {
                counter_ = 0;
                descend_ = false;
            }


            using visitor::Base<Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }


            template <class t_Ariles>
            void startRoot(const t_Ariles &, const Parameters &)
            {
                ARILES_TRACE_FUNCTION;
                counter_ = 0;
                descend_ = true;
            }

            using visitor::Base<Parameters>::finishRoot;


            template <class t_Entry>
            void operator()(
                    const t_Entry &entry,
                    const std::string &name,
                    const Parameters & /*param*/,
                    ARILES_IS_BASE_DISABLER(entry::ConstBase<count::Visitor>, t_Entry))
            {
                ARILES_UNUSED_ARG(name);
                ARILES_UNUSED_ARG(entry);
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_ENTRY(name);
                ARILES_TRACE_TYPE(entry);
                ++this->counter_;
            }


            template <class t_Entry>
            void operator()(
                    const t_Entry &entry,
                    const std::string &name,
                    const Parameters &param,
                    ARILES_IS_BASE_ENABLER(entry::ConstBase<count::Visitor>, t_Entry))
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



        class ARILES_VISIBILITY_ATTRIBUTE Base : public entry::ConstBase<count::Visitor>
        {
        public:
        };

#ifndef ARILES_METHODS_count
#    define ARILES_METHODS_count ARILES_METHODS(count, ARILES_EMPTY_MACRO, const)
#endif
    }  // namespace count


    typedef count::Visitor Count;
}  // namespace ariles
