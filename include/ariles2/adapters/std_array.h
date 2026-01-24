/**
    @file
    @author  Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <array>
#include "../internal/helpers.h"


namespace ariles2
{
    template <class t_Type, std::size_t t_Size>
    bool isMissing(const std::array<t_Type, t_Size> &entry)
    {
        return (entry.empty());
    }
}  // namespace ariles2


namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, class t_Type, std::size_t t_Size>
        void apply_read(t_Visitor &visitor, std::array<t_Type, t_Size> &entry, const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            const std::size_t size = visitor.startArray();
            if (size != t_Size)
            {
                throw std::runtime_error("Array size mismatch: expected " + std::to_string(t_Size) + ", got " + std::to_string(size));
            }
            for (std::size_t i = 0; i < t_Size; ++i)
            {
                visitor.visitArrayElement(entry[i], param);
            }
            visitor.endArray();
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, class t_Type, std::size_t t_Size>
        void apply_write(
                t_Visitor &writer,
                const std::array<t_Type, t_Size> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            writer.startArray(t_Size, param.compact_arrays_);
            for (const t_Type &value : entry)
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
        template <class t_Visitor, class t_Type, std::size_t t_Size>
        void apply_compare(
                t_Visitor &visitor,
                const std::array<t_Type, t_Size> &left,
                const std::array<t_Type, t_Size> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            for (std::size_t i = 0; i < t_Size; ++i)
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
        template <class t_Visitor, class t_Type, std::size_t t_Size>
        void apply_defaults(
                const t_Visitor &visitor,
                std::array<t_Type, t_Size> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            for (t_Type &element : entry)
            {
                apply_defaults(visitor, element, param);
            }
        }
    }  // namespace defaults
}  // namespace ariles2



namespace ariles2
{
    namespace process
    {
        template <class t_Visitor, class t_Type, std::size_t t_Size>
        void apply_process(
                const t_Visitor &visitor,
                std::array<t_Type, t_Size> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            for (typename std::array<t_Type, t_Size>::reference element : entry)
            {
                apply_process(visitor, element, param);
            }
        }
    }  // namespace process
}  // namespace ariles2


namespace ariles2
{
    namespace copyfrom
    {
        template <class t_Visitor, class t_LeftType, class t_RightType, std::size_t t_Size>
        void apply_copyfrom(
                t_Visitor &visitor,
                std::array<t_LeftType, t_Size> &left,
                const std::array<t_RightType, t_Size> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            for (std::size_t i = 0; i < t_Size; ++i)
            {
                apply_copyfrom(visitor, left[i], right[i], param);
            }
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <class t_Visitor, class t_LeftType, class t_RightType, std::size_t t_Size>
        void apply_copyto(
                t_Visitor &visitor,
                const std::array<t_LeftType, t_Size> &left,
                std::array<t_RightType, t_Size> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;

            for (std::size_t i = 0; i < t_Size; ++i)
            {
                apply_copyto(visitor, left[i], right[i], param);
            }
        }
    }  // namespace copyto
}  // namespace ariles2
