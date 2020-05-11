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
    namespace process
    {
        template <class t_Derived, class t_Parameters>
        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public ariles2::visitor::Base<visitor::GenericVisitor, t_Parameters>
        {
        public:
            typedef t_Parameters Parameters;


        public:
            using visitor::Base<visitor::GenericVisitor, t_Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const t_Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*(static_cast<t_Derived *>(this))));
            }


            template <class t_Entry>
            void start(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                ARILES_TRACE_FUNCTION;
                this->operator()(entry, name, param);
            }


            template <class t_Entry>
            void operator()(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                ARILES2_UNUSED_ARG(name);
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_ENTRY(name);
                ARILES_TRACE_TYPE(entry);
                apply_process(*(static_cast<t_Derived *>(this)), entry, param);
            }
        };
    }  // namespace process
}  // namespace ariles2
