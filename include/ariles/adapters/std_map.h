/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <map>

namespace ariles
{
    template <  class t_Reader,
                typename t_Key,
                typename t_Value,
                class t_Compare,
                class t_Allocator>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                t_Reader & reader,
                std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                const typename t_Reader::Parameters & param)
    {
        std::size_t size = reader.startArray();
        typename t_Reader::Parameters param_local = param;
        param_local.unset(t_Reader::Parameters::ALLOW_MISSING_ENTRIES);
        entry.clear();
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
                const typename t_Reader::Parameters & param)
    {
        if (reader.getBridgeFlags().isSet(BridgeFlags::SLOPPY_MAPS_SUPPORTED)
                && param.isSet(t_Reader::Parameters::SLOPPY_MAPS_IF_SUPPORTED))
        {
            std::vector<std::string> entry_names;
            ARILES_ASSERT(true == reader.getMapEntryNames(entry_names), "Could not read names of map entries.");
            typename t_Reader::Parameters param_local = param;
            param_local.unset(t_Reader::Parameters::ALLOW_MISSING_ENTRIES);
            entry.clear();
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



    template <  class t_Writer,
                typename t_Key,
                typename t_Value,
                class t_Compare,
                class t_Allocator>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                t_Writer & writer,
                const std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                const typename t_Writer::Parameters & param)
    {
        writer.startArray(entry.size(), param.isSet(t_Writer::Parameters::COMPACT_ARRAYS_IF_SUPPORTED));
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
                const typename t_Writer::Parameters & param)
    {
        if (writer.getBridgeFlags().isSet(BridgeFlags::SLOPPY_MAPS_SUPPORTED)
                && param.isSet(t_Writer::Parameters::SLOPPY_MAPS_IF_SUPPORTED))
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


namespace ariles
{
    namespace compare
    {
        template <  class t_Iterator,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> &left,
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> &right,
                    const typename t_Iterator::CompareParameters & param)
        {
            ARILES_TRACE_FUNCTION;

            if (left.size() != right.size())
            {
                return (false);
            }

            typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator left_it = left.begin();
            typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator right_it = right.begin();

            for (; (left_it != left.end()) && (right_it != right.end()); ++left_it, ++right_it)
            {
                if (false == apply(iterator, left_it->first, right_it->first, param))
                {
                    return (false);
                }

                if (false == apply(iterator, left_it->second, right_it->second, param))
                {
                    return (false);
                }
            }

            return (true);
        }
    }
}



namespace ariles
{
    namespace defaults
    {
        template <  class t_Iterator,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & /*iterator*/,
                    std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const typename t_Iterator::DefaultsParameters & /*param*/)
        {
            ARILES_TRACE_FUNCTION;
            entry.clear();
        }
    }
}


namespace ariles
{
    namespace finalize
    {
        template <  class t_Iterator,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    std::map<t_Key, t_Value, t_Compare, t_Allocator> &entry,
                    const typename t_Iterator::FinalizeParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            for (
                typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::iterator it = entry.begin();
                it != entry.end();
                ++it)
            {
                apply(iterator, it->first, param);
                apply(iterator, it->second, param);
            }
        }
    }
}

