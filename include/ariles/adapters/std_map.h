/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_INCLUDED_ADAPTER_STD_MAP

#include <map>

namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const bool crash_on_missing_entry = false);
    }


    namespace writer
    {
        template <  class t_Writer,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                    t_Writer & writer,
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry);
    }
}
