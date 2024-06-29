/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

/**
@defgroup process Process

@brief Process entries, base for @ref prewrite and @ref finalize.
*/

namespace ariles2
{
    namespace process
    {
        template <class t_Derived, class t_Parameters>
        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public ariles2::visitor::Base<t_Derived, t_Parameters>
        {
        public:
            using Parameters = t_Parameters;


        public:
            template <class t_Entry>
            void visit(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                CPPUT_TRACE_FUNCTION;
                this->visitMapEntry(entry, name, param);
            }


            template <class t_Entry>
            void visitMapEntry(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                CPPUT_UNUSED_ARG(name);
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(name);
                CPPUT_TRACE_TYPE(entry);
                apply_process(*(static_cast<t_Derived *>(this)), entry, param);
            }
        };
    }  // namespace process
}  // namespace ariles2
