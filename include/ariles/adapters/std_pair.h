/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <utility>

namespace ariles
{
    template <  class t_Reader,
                typename t_First,
                typename t_Second,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                t_Reader & reader,
                std::pair<t_First, t_Second> & entry,
                const t_Flags & param)
    {
        ariles::ConfigurableFlags param_local = param;
        param_local.unset(ConfigurableFlags::ALLOW_MISSING_ENTRIES);
        reader.template startMap<t_Reader::SIZE_LIMIT_EQUAL>(2);
        readEntry(reader, entry.first, "first", param_local);
        readEntry(reader, entry.second, "second", param_local);
        reader.endMap();
    }



    template <  class t_Writer,
                typename t_First,
                typename t_Second,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                t_Writer & writer,
                const std::pair<t_First, t_Second> & entry,
                const t_Flags & param)
    {
        writer.startMap(2);
        writeEntry(writer, entry.first, "first", param);
        writeEntry(writer, entry.second, "second", param);
        writer.endMap();
    }



    template <  typename t_First,
                typename t_Second,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(
                std::pair<t_First, t_Second> & entry,
                const t_Flags & param)
    {
        setDefaults(entry.first, param);
        setDefaults(entry.second, param);
    }
}
