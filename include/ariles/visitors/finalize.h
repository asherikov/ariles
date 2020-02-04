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
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::Visitor
        {
            public:
                class Parameters
                {
                };


            public:
                const Parameters & getDefaultParameters() const
                {
                    const static Parameters parameters;
                    return parameters;
                }


                template<class t_Configurable>
                    void startRoot( const t_Configurable &,
                                    const Parameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Configurable>
                    void finishRoot(t_Configurable &,
                                    const Parameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }
        };


        template<class t_Derived>
            class ARILES_VISIBILITY_ATTRIBUTE Base
                : public visitor::Base<t_Derived, const finalize::Visitor>
        {
            public:
        };
    }
}
