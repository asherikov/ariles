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
    namespace read
    {
        template <  class t_Visitor,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                    t_Visitor & visitor,
                    std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            std::size_t size = visitor.startArray();
            typename t_Visitor::Parameters param_local = param;
            param_local.unset(t_Visitor::Parameters::ALLOW_MISSING_ENTRIES);
            entry.clear();
            for(std::size_t i = 0; i < size; ++i)
            {
                std::pair<t_Key, t_Value> map_entry;

                apply_read(visitor, map_entry, param_local);

                entry.insert(map_entry);

                visitor.shiftArray();
            }
            visitor.endArray();
        }


        template <  class t_Visitor,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                    t_Visitor & visitor,
                    std::map<std::string, t_Value, t_Compare, t_Allocator> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (visitor.getBridgeFlags().isSet(BridgeFlags::SLOPPY_MAPS_SUPPORTED)
                    && param.isSet(t_Visitor::Parameters::SLOPPY_MAPS_IF_SUPPORTED))
            {
                std::vector<std::string> entry_names;
                ARILES_ASSERT(true == visitor.getMapEntryNames(entry_names), "Could not read names of map entries.");
                typename t_Visitor::Parameters param_local = param;
                param_local.unset(t_Visitor::Parameters::ALLOW_MISSING_ENTRIES);
                entry.clear();
                visitor.template startMap<t_Visitor::SIZE_LIMIT_NONE>();
                for (std::size_t i = 0; i < entry_names.size(); ++i)
                {
                    t_Value entry_value;
                    applyToEntry(visitor, entry_value, entry_names[i], param_local);
                    entry[entry_names[i]] = entry_value;
                }
                visitor.endMap();
            }
            else
            {
                apply_read<t_Visitor, std::string, t_Value, t_Compare, t_Allocator>(visitor, entry, param);
            }
        }
    }
}


namespace ariles
{
    namespace write
    {
        template <  class t_Visitor,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                    t_Visitor & writer,
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            writer.startArray(entry.size(), param.isSet(t_Visitor::Parameters::COMPACT_ARRAYS_IF_SUPPORTED));
            for (
                typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                it != entry.end();
                ++it)
            {
                apply_write(writer, *it, param);
                writer.shiftArray();
            }
            writer.endArray();
        }


        template <  class t_Visitor,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                    t_Visitor & writer,
                    const std::map<std::string, t_Value, t_Compare, t_Allocator> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (writer.getBridgeFlags().isSet(BridgeFlags::SLOPPY_MAPS_SUPPORTED)
                    && param.isSet(t_Visitor::Parameters::SLOPPY_MAPS_IF_SUPPORTED))
            {
                writer.startMap(entry.size());
                for (
                    typename std::map<std::string, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                    it != entry.end();
                    ++it)
                {
                    applyToEntry(writer, it->second, it->first, param);
                }
                writer.endMap();
            }
            else
            {
                apply_write<t_Visitor, std::string, t_Value, t_Compare, t_Allocator>(writer, entry, param);
            }
        }
    }
}


namespace ariles
{
    namespace compare
    {
        template <  class t_Visitor,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                    t_Visitor & visitor,
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> &left,
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> &right,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;

            visitor.equal_ &= (left.size() == right.size());

            typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator left_it = left.begin();
            typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator right_it = right.begin();

            for (; (left_it != left.end()) and (right_it != right.end()) and (true == visitor.equal_); ++left_it, ++right_it)
            {
                apply_compare(visitor, left_it->first, right_it->first, param);
                apply_compare(visitor, left_it->second, right_it->second, param);
            }
        }
    }
}



namespace ariles
{
    namespace defaults
    {
        template <  class t_Visitor,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                    const t_Visitor & /*visitor*/,
                    std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const typename t_Visitor::Parameters & /*param*/)
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
        template <  class t_Visitor,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply_finalize(
                    const t_Visitor & visitor,
                    std::map<t_Key, t_Value, t_Compare, t_Allocator> &entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            for (
                typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::iterator it = entry.begin();
                it != entry.end();
                ++it)
            {
                apply_finalize(visitor, it->first, param);
                apply_finalize(visitor, it->second, param);
            }
        }
    }
}

