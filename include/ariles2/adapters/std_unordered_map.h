/**
    @file
    @author  Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <unordered_map>
#include <utility>

#include "../internal/helpers.h"
#include "../visitors/serialization.h"
#include "std_pair.h"


namespace ariles2
{
    template <class... t_Args>
    bool isMissing(const std::unordered_map<t_Args...> &entry)
    {
        return (entry.empty());
    }
}  // namespace ariles2


namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, class... t_Args>
        void apply_read(t_Visitor &visitor, std::unordered_map<t_Args...> &entry, const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            const std::size_t size = visitor.startArray();
            entry.clear();
            for (std::size_t i = 0; i < size; ++i)
            {
                std::pair<typename std::unordered_map<t_Args...>::key_type, typename std::unordered_map<t_Args...>::mapped_type> map_entry;

                visitor.startArrayElement();
                apply_read(visitor, map_entry, param);
                entry.insert(map_entry);
                visitor.endArrayElement();
            }
            visitor.endArray();
        }


        template <class t_Visitor, class... t_Args>
        void apply_read(
                t_Visitor &visitor,
                std::unordered_map<std::string, t_Args...> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            CPPUT_TRACE_FUNCTION;
            if (parameters.sloppy_maps_ and visitor.startIteratedMap(t_Visitor::SIZE_LIMIT_MIN, 1))
            {
                entry.clear();
                std::string entry_name;
                while (visitor.startIteratedMapElement(entry_name))
                {
                    apply_read(visitor, entry[entry_name], parameters);
                    visitor.endIteratedMapElement();
                }
                visitor.endIteratedMap();
            }
            else
            {
                apply_read<t_Visitor, std::string, t_Args...>(visitor, entry, parameters);
            }
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, class... t_Args>
        void apply_write(
                t_Visitor &writer,
                const std::unordered_map<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            writer.startArray(entry.size(), param.compact_arrays_);
            for (const typename std::unordered_map<t_Args...>::value_type &value : entry)
            {
                writer.visitArrayElement(value, param);
            }
            writer.endArray();
        }


        template <class t_Visitor, class... t_Args>
        void apply_write(
                t_Visitor &writer,
                const std::unordered_map<std::string, t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            if (param.sloppy_maps_)
            {
                if (writer.startIteratedMap(entry.size(), param))
                {
                    for (const typename std::unordered_map<std::string, t_Args...>::value_type &value : entry)
                    {
                        writer.startIteratedMapElement(value.first);
                        apply_write(writer, value.second, param);
                        writer.endIteratedMapElement();
                    }
                    writer.endIteratedMap();
                    return;
                }
            }
            apply_write<t_Visitor, std::string, t_Args...>(writer, entry, param);
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, class... t_Args>
        void apply_compare(
                t_Visitor &visitor,
                const std::unordered_map<t_Args...> &left,
                const std::unordered_map<t_Args...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            visitor.equal_ &= (left.size() == right.size());

            if (visitor.equal_ && !left.empty())
            {
                for (const typename std::unordered_map<t_Args...>::value_type &left_pair : left)
                {
                    typename std::unordered_map<t_Args...>::const_iterator right_it = right.find(left_pair.first);
                    if (right_it == right.end())
                    {
                        visitor.equal_ = false;
                        break;
                    }
                    apply_compare(visitor, left_pair.second, right_it->second, param);
                }
            }
        }
    }  // namespace compare
}  // namespace ariles2



namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, class... t_Args>
        void apply_defaults(
                const t_Visitor & /*visitor*/,
                std::unordered_map<t_Args...> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            entry.clear();
        }
    }  // namespace defaults
}  // namespace ariles2


namespace ariles2
{
    namespace process
    {
        template <class t_Visitor, class... t_Args>
        void apply_process(
                const t_Visitor &visitor,
                std::unordered_map<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            for (typename std::unordered_map<t_Args...>::reference value : entry)
            {
                apply_process(visitor, value.first, param);
                apply_process(visitor, value.second, param);
            }
        }
    }  // namespace process
}  // namespace ariles2


namespace ariles2
{
    namespace copyfrom
    {
        template <class t_Visitor, class... t_ArgsLeft, class... t_ArgsRight>
        void apply_copyfrom(
                t_Visitor &visitor,
                std::unordered_map<t_ArgsLeft...> &left,
                const std::unordered_map<t_ArgsRight...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            left.clear();

            for (const typename std::unordered_map<t_ArgsRight...>::value_type &right_pair : right)
            {
                typename std::unordered_map<t_ArgsLeft...>::key_type left_key;
                apply_copyfrom(visitor, left_key, right_pair.first, param);
                apply_copyfrom(visitor, left[left_key], right_pair.second, param);
            }
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <class t_Visitor, class... t_ArgsLeft, class... t_ArgsRight>
        void apply_copyto(
                t_Visitor &visitor,
                const std::unordered_map<t_ArgsLeft...> &left,
                std::unordered_map<t_ArgsRight...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            right.clear();

            for (const typename std::unordered_map<t_ArgsLeft...>::value_type &left_pair : left)
            {
                typename std::unordered_map<t_ArgsRight...>::key_type right_key;
                apply_copyto(visitor, left_pair.first, right_key, param);
                apply_copyto(visitor, left_pair.second, right[right_key], param);
            }
        }
    }  // namespace copyto
}  // namespace ariles2
