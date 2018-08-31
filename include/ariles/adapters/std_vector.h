/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_INCLUDED_ADAPTER_STD_VECTOR

#include <vector>

namespace ariles
{
    namespace adapter
    {
        template <  class t_Reader,
                    typename t_VectorEntryType,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::vector<t_VectorEntryType, t_Allocator> & entry,
                    const ariles::ConfigurableFlags & param);



        template <  class t_Writer,
                    typename t_VectorEntryType,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                    t_Writer & writer,
                    const std::vector<t_VectorEntryType, t_Allocator> & entry,
                    const ariles::ConfigurableFlags & param);



        template <  typename t_VectorEntryType,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE setDefaults(
                    std::vector<t_VectorEntryType, t_Allocator> & entry);
    }
}
