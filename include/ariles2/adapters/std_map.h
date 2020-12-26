/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <map>

#include "../internal/helpers.h"
#include "../visitors/serialization.h"
#include "std_pair.h"

namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, typename t_Key, typename t_Value, class t_Compare, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                std::map<t_Key, t_Value, t_Compare, t_Allocator> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            std::size_t size = visitor.startArray();
            entry.clear();
            for (std::size_t i = 0; i < size; ++i)
            {
                std::pair<t_Key, t_Value> map_entry;

                visitor.startArrayElement();
                apply_read(visitor, map_entry, param);
                entry.insert(map_entry);
                visitor.endArrayElement();
            }
            visitor.endArray();
        }


        template <class t_Visitor, typename t_Value, class t_Compare, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                std::map<std::string, t_Value, t_Compare, t_Allocator> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == parameters.sloppy_maps_ and true == visitor.startIteratedMap(t_Visitor::SIZE_LIMIT_MIN, 1))
            {
                entry.clear();
                std::string entry_name;
                while (true == visitor.startIteratedMapElement(entry_name))
                {
                    apply_read(visitor, entry[entry_name], parameters);
                    visitor.endIteratedMapElement();
                }
                visitor.endIteratedMap();
            }
            else
            {
                apply_read<t_Visitor, std::string, t_Value, t_Compare, t_Allocator>(visitor, entry, parameters);
            }
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, typename t_Key, typename t_Value, class t_Compare, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const std::map<t_Key, t_Value, t_Compare, t_Allocator> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            writer.startArray(entry.size(), param.compact_arrays_);
            for (typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                 it != entry.end();
                 ++it)
            {
                writer.visitArrayElement(*it, param);
            }
            writer.endArray();
        }


        template <class t_Visitor, typename t_Value, class t_Compare, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const std::map<std::string, t_Value, t_Compare, t_Allocator> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == param.sloppy_maps_)
            {
                if (true == writer.startIteratedMap(entry.size(), param))
                {
                    for (typename std::map<std::string, t_Value, t_Compare, t_Allocator>::const_iterator it =
                                 entry.begin();
                         it != entry.end();
                         ++it)
                    {
                        writer.startIteratedMapElement(it->first);
                        apply_write(writer, it->second, param);
                        writer.endIteratedMapElement();
                    }
                    writer.endIteratedMap();
                    return;
                }
            }
            apply_write<t_Visitor, std::string, t_Value, t_Compare, t_Allocator>(writer, entry, param);
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, typename t_Key, typename t_Value, class t_Compare, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const std::map<t_Key, t_Value, t_Compare, t_Allocator> &left,
                const std::map<t_Key, t_Value, t_Compare, t_Allocator> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            visitor.equal_ &= (left.size() == right.size());

            typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator left_it = left.begin();
            typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator right_it = right.begin();

            for (; (left_it != left.end()) and (right_it != right.end()) and (true == visitor.equal_);
                 ++left_it, ++right_it)
            {
                apply_compare(visitor, left_it->first, right_it->first, param);
                apply_compare(visitor, left_it->second, right_it->second, param);
            }
        }
    }  // namespace compare
}  // namespace ariles2



namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, typename t_Key, typename t_Value, class t_Compare, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                std::map<t_Key, t_Value, t_Compare, t_Allocator> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            entry.clear();
        }
    }  // namespace defaults
}  // namespace ariles2


namespace ariles2
{
    namespace process
    {
        template <class t_Visitor, typename t_Key, typename t_Value, class t_Compare, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_process(
                const t_Visitor &visitor,
                std::map<t_Key, t_Value, t_Compare, t_Allocator> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            for (typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::iterator it = entry.begin();
                 it != entry.end();
                 ++it)
            {
                apply_process(visitor, it->first, param);
                apply_process(visitor, it->second, param);
            }
        }
    }  // namespace process
}  // namespace ariles2


namespace ariles2
{
    namespace copyfrom
    {
        template <
                class t_Visitor,
                typename t_KeyLeft,
                typename t_ValueLeft,
                class t_CompareLeft,
                class t_AllocatorLeft,
                typename t_KeyRight,
                typename t_ValueRight,
                class t_CompareRight,
                class t_AllocatorRight>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyfrom(
                t_Visitor &visitor,
                std::map<t_KeyLeft, t_ValueLeft, t_CompareLeft, t_AllocatorLeft> &left,
                const std::map<t_KeyRight, t_ValueRight, t_CompareRight, t_AllocatorRight> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            typename std::map<t_KeyRight, t_ValueRight, t_CompareRight, t_AllocatorRight>::const_iterator right_it =
                    right.begin();

            left.clear();

            for (; right_it != right.end(); ++right_it)
            {
                t_KeyLeft left_key;

                apply_copyfrom(visitor, left_key, right_it->first, param);
                apply_copyfrom(visitor, left[left_key], right_it->second, param);
            }
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <
                class t_Visitor,
                typename t_KeyLeft,
                typename t_ValueLeft,
                class t_CompareLeft,
                class t_AllocatorLeft,
                typename t_KeyRight,
                typename t_ValueRight,
                class t_CompareRight,
                class t_AllocatorRight>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                t_Visitor &visitor,
                const std::map<t_KeyLeft, t_ValueLeft, t_CompareLeft, t_AllocatorLeft> &left,
                std::map<t_KeyRight, t_ValueRight, t_CompareRight, t_AllocatorRight> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            typename std::map<t_KeyLeft, t_ValueLeft, t_CompareLeft, t_AllocatorLeft>::const_iterator left_it =
                    left.begin();

            right.clear();

            for (; left_it != left.end(); ++left_it)
            {
                t_KeyRight right_key;

                apply_copyto(visitor, left_it->first, right_key, param);
                apply_copyto(visitor, left_it->second, right[right_key], param);
            }
        }
    }  // namespace copyto
}  // namespace ariles2
