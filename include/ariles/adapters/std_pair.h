/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_INCLUDED_ADAPTER_STD_PAIR

#include <utility>

namespace ariles
{
    namespace adapter
    {
        template <  class t_Reader,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::pair<t_First, t_Second> & entry,
                    const ariles::ConfigurableFlags & param);



        template <  class t_Writer,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                    t_Writer & writer,
                    const std::pair<t_First, t_Second> & entry,
                    const ariles::ConfigurableFlags & param);


        template <  typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE setDefaults(
                    std::pair<t_First, t_Second> & entry);
    }
}
