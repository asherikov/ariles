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
    namespace finalize
    {
        class Parameters
        {
        };


        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::VisitorBase<finalize::Parameters>
        {
            public:
                typedef finalize::Parameters Parameters;


            public:
                using visitor::VisitorBase<Parameters>::getDefaultParameters;

                template<class t_Ariles>
                    const Parameters & getParameters(const t_Ariles & ariles_class) const
                {
                    return (ariles_class.arilesGetParameters(*this));
                }

                using visitor::VisitorBase<Parameters>::startRoot;
                using visitor::VisitorBase<Parameters>::finishRoot;


                template<class t_Entry>
                    void operator()(
                            t_Entry & entry,
                            const std::string & name,
                            const Parameters & param) const
                {
                    ARILES_UNUSED_ARG(name);
                    ARILES_TRACE_FUNCTION;
                    ARILES_TRACE_ENTRY(name);
                    ARILES_TRACE_TYPE(entry);
                    apply_finalize(*this, entry, param);
                }
        };


        class ARILES_VISIBILITY_ATTRIBUTE Base
            : public visitor::Base<const finalize::Visitor>
        {
            public:
        };


#ifndef ARILES_METHODS_finalize
#   define ARILES_METHODS_finalize ARILES_METHODS(const ariles::finalize::Visitor, ARILES_EMPTY_MACRO)
#endif
    }
}
