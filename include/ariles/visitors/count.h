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
                class CountParameters
                {
                } default_parameters_;


            public:
                std::size_t counter_;
                bool descend_;


            public:
                Visitor()
                {
                    counter_ = 0;
                    descend_ = false;
                }


                template<class t_Configurable>
                    void startRoot( const t_Configurable &,
                                    const CountParameters &)
                {
                    ARILES_TRACE_FUNCTION;
                    counter_ = 0;
                    descend_ = true;
                }


                template<class t_Configurable>
                    void finishRoot(const t_Configurable &,
                                    const CountParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }
        };



        template<class t_Derived>
            class ARILES_VISIBILITY_ATTRIBUTE Base
        {
            public:
        };
    }
}
