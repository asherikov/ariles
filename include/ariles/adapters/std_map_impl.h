/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <iostream>
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
                    const ariles::ConfigurableFlags & param)
        {
            std::size_t size = reader.startArray();
            entry.clear();
            ariles::ConfigurableFlags param_local = param;
            param_local.set(ConfigurableFlags::CRASH_ON_MISSING_ENTRY);
            for(std::size_t i = 0; i < size; ++i)
            {
                std::pair<t_Key, t_Value> map_entry;

                readBody(reader, map_entry, param_local);

                entry.insert(map_entry);

                reader.shiftArray();
            }
            reader.endArray();
        }


        template <  class t_Reader,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::map<std::string, t_Value, t_Compare, t_Allocator> & entry,
                    const ariles::ConfigurableFlags & param)
        {
            if (reader.getBridgeFlags().isSet(BridgeFlags::SLOPPY_MAPS_SUPPORTED)
                    && param.isSet(ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED))
            {
                std::vector<std::string> entry_names;
                ARILES_ASSERT(true == reader.getMapEntryNames(entry_names), "Could not read names of map entries.");
                entry.clear();
                ariles::ConfigurableFlags param_local = param;
                param_local.set(ConfigurableFlags::CRASH_ON_MISSING_ENTRY);
                reader.template startMap<t_Reader::SIZE_LIMIT_NONE>();
                for (std::size_t i = 0; i < entry_names.size(); ++i)
                {
                    t_Value entry_value;
                    readEntry(reader, entry_value, entry_names[i], param_local);
                    entry[entry_names[i]] = entry_value;
                }
                reader.endMap();
            }
            else
            {
                readBody<t_Reader, std::string, t_Value, t_Compare, t_Allocator>(reader, entry, param);
            }
        }
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
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const ariles::ConfigurableFlags & param)
        {
            writer.startArray(entry.size(), param.isSet(ConfigurableFlags::COMPACT_ARRAYS_IF_SUPPORTED));
            for (
                typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                it != entry.end();
                ++it)
            {
                writeBody(writer, *it, param);
                writer.shiftArray();
            }
            writer.endArray();
        }


        template <  class t_Writer,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                    t_Writer & writer,
                    const std::map<std::string, t_Value, t_Compare, t_Allocator> & entry,
                    const ariles::ConfigurableFlags & param)
        {
            if (writer.getBridgeFlags().isSet(BridgeFlags::SLOPPY_MAPS_SUPPORTED)
                    && param.isSet(ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED))
            {
                writer.startMap(entry.size());
                for (
                    typename std::map<std::string, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                    it != entry.end();
                    ++it)
                {
                    writeEntry(writer, it->second, it->first, param);
                }
                writer.endMap();
            }
            else
            {
                writeBody<t_Writer, std::string, t_Value, t_Compare, t_Allocator>(writer, entry, param);
            }
        }
    }
}
