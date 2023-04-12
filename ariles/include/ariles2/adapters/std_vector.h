/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <vector>
#include "../internal/helpers.h"


namespace ariles2
{
    template <class... t_Args>
    bool isMissing(const std::vector<t_Args...> &entry)
    {
        return (entry.empty());
    }
}  // namespace ariles2


namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, class... t_Args>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                std::vector<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            entry.resize(visitor.startArray());
            for (typename std::vector<t_Args...>::value_type &value : entry)
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
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const std::vector<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            writer.startArray(entry.size(), param.compact_arrays_);
            for (const typename std::vector<t_Args...>::value_type &value : entry)
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
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const std::vector<t_Args...> &left,
                const std::vector<t_Args...> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            visitor.equal_ &= (left.size() == right.size());

            for (std::size_t i = 0; i < left.size() and visitor.equal_; ++i)
            {
                apply_compare(visitor, left[i], right[i], param);
            }
        }
    }  // namespace compare
}  // namespace ariles2


namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, class... t_Args>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                std::vector<t_Args...> &entry,
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
        template <class t_Visitor, class... t_Args>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_process(
                const t_Visitor &visitor,
                std::vector<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            for (typename std::vector<t_Args...>::value_type &value : entry)
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
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyfrom(
                t_Visitor &visitor,
                std::vector<t_LeftArgs...> &left,
                const std::vector<t_RightArgs...> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            left.clear();
            left.reserve(right.size());
            for (const typename std::vector<t_RightArgs...>::value_type &right_value : right)
            {
                typename std::vector<t_LeftArgs...>::value_type left_value;
                apply_copyfrom(visitor, left_value, right_value, param);
                left.push_back(std::move(left_value));
            }
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <class t_Visitor, class... t_LeftArgs, class... t_RightArgs>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                t_Visitor &visitor,
                const std::vector<t_LeftArgs...> &left,
                std::vector<t_RightArgs...> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            right.clear();
            right.reserve(left.size());
            for (const typename std::vector<t_LeftArgs...>::value_type &left_value : left)
            {
                typename std::vector<t_RightArgs...>::value_type right_value;
                apply_copyto(visitor, left_value, right_value, param);
                right.push_back(std::move(right_value));
            }
        }
    }  // namespace copyto
}  // namespace ariles2
