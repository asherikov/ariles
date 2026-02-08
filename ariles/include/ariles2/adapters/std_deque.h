/**
    @file
    @author  Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <deque>
#include "../internal/helpers.h"


namespace ariles2
{
    template <class... t_Args>
    bool isMissing(const std::deque<t_Args...> &entry)
    {
        return (entry.empty());
    }
}  // namespace ariles2


namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, class... t_Args>
        void apply_read(t_Visitor &visitor, std::deque<t_Args...> &entry, const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            const std::size_t size = visitor.startArray();
            entry.clear();
            entry.resize(size);
            for (typename std::deque<t_Args...>::reference value : entry)
            {
                visitor.visitArrayElement(value, param);
            }
            visitor.endArray();
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
                const std::deque<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            writer.startArray(entry.size(), param.compact_arrays_);
            for (const typename std::deque<t_Args...>::value_type &value : entry)
            {
                writer.visitArrayElement(value, param);
            }
            writer.endArray();
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
                const std::deque<t_Args...> &left,
                const std::deque<t_Args...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            visitor.equal_ &= (left.size() == right.size());

            typename std::deque<t_Args...>::const_iterator left_it = left.begin();
            typename std::deque<t_Args...>::const_iterator right_it = right.begin();

            for (; left_it != left.end() && right_it != right.end(); ++left_it, ++right_it)
            {
                apply_compare(visitor, *left_it, *right_it, param);
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
                std::deque<t_Args...> &entry,
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
                std::deque<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            for (typename std::deque<t_Args...>::reference value : entry)
            {
                apply_process(visitor, value, param);
            }
        }
    }  // namespace process
}  // namespace ariles2


namespace ariles2
{
    namespace copyfrom
    {
        template <class t_Visitor, class... t_LeftArgs, class... t_RightArgs>
        void apply_copyfrom(
                t_Visitor &visitor,
                std::deque<t_LeftArgs...> &left,
                const std::deque<t_RightArgs...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            left.clear();
            for (const typename std::deque<t_RightArgs...>::value_type &right_value : right)
            {
                typename std::deque<t_LeftArgs...>::value_type left_value;
                apply_copyfrom(visitor, left_value, right_value, param);
                left.push_back(std::move(left_value));
            }
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <class t_Visitor, class... t_LeftArgs, class... t_RightArgs>
        void apply_copyto(
                t_Visitor &visitor,
                const std::deque<t_LeftArgs...> &left,
                std::deque<t_RightArgs...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            right.clear();
            for (const typename std::deque<t_LeftArgs...>::value_type &left_value : left)
            {
                typename std::deque<t_RightArgs...>::value_type right_value;
                apply_copyto(visitor, left_value, right_value, param);
                right.push_back(std::move(right_value));
            }
        }
    }  // namespace copyto
}  // namespace ariles2
