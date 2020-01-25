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
                bool descend_;


            public:
                Iterator()
                {
                    counter_ = 0;
                    descend_ = false;
                }


                template<class t_Configurable>
                    void startBody( const t_Configurable &,
                                    const CountParameters &)
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Configurable>
                    void finishBody(const t_Configurable &,
                                    const CountParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Configurable>
                    void startRoot( const t_Configurable &,
                                    const CountParameters &)
                {
                    ARILES_TRACE_FUNCTION;
                    descend_ = true;
                }


                template<class t_Configurable>
                    void finishRoot(const t_Configurable &,
                                    const CountParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }
        };



        class ARILES_VISIBILITY_ATTRIBUTE Base
        {
            public:
                /**
                 * @brief Get number of entries in the corresponding
                 * configuration node.
                 *
                 * @return number of entries
                 *
                 * @todo DEPRECATED
                 */
                virtual std::size_t getNumberOfEntries() const = 0;
        };
    }
}
