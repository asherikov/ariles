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
    namespace process
    {
        template <class t_Derived, class t_Parameters>
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::Base<t_Parameters>
        {
        public:
            typedef t_Parameters Parameters;


        public:
            using visitor::Base<t_Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const t_Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*(static_cast<t_Derived *>(this))));
            }

            using visitor::Base<t_Parameters>::startRoot;
            using visitor::Base<t_Parameters>::finishRoot;


            template <class t_Entry>
            void operator()(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                ARILES_UNUSED_ARG(name);
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_ENTRY(name);
                ARILES_TRACE_TYPE(entry);
                apply_process(*(static_cast<t_Derived *>(this)), entry, param);
            }
        };
    }  // namespace process
}  // namespace ariles
