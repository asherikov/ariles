/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <limits>

namespace ariles
{
    namespace count
    {
        class ARILES_VISIBILITY_ATTRIBUTE Iterator
        {
            public:
                class CountParameters
                {
                } default_parameters_;


            public:
                std::size_t counter_;


            public:
                Iterator()
                {
                    counter_ = 0;
                }


                template<class t_Configurable>
                    void start( const t_Configurable &,
                                const CountParameters &)
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Configurable>
                    void finish(const t_Configurable &,
                                const CountParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }
        };



        class Base
        {
            public:
        };


        template<   class t_Iterator,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Iterator & iterator,
                    const t_Entry & /*entry*/,
                    const std::string & /*name*/,
                    const typename t_Iterator::CountParameters & /*param*/)
        {
            ARILES_TRACE_FUNCTION;
            ++iterator.counter_;
        }
    }
}
