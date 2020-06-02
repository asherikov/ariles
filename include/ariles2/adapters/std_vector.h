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
    namespace read
    {
        template <class t_Visitor, typename t_VectorEntryType, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                std::vector<t_VectorEntryType, t_Allocator> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            entry.resize(visitor.startArray());
            for (std::size_t i = 0; i < entry.size(); ++i)
            {
                apply_read(visitor, entry[i], param);
                visitor.shiftArray();
            }
            visitor.endArray();
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, typename t_VectorEntryType, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const std::vector<t_VectorEntryType, t_Allocator> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            writer.startArray(entry.size(), param.compact_arrays_);
            for (std::size_t i = 0; i < entry.size(); ++i)
            {
                apply_write(writer, entry[i], param);
                writer.shiftArray();
            }
            writer.endArray();
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, typename t_VectorEntryType, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const std::vector<t_VectorEntryType, t_Allocator> &left,
                const std::vector<t_VectorEntryType, t_Allocator> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            visitor.equal_ &= (left.size() == right.size());

            for (std::size_t i = 0; i < left.size() and true == visitor.equal_; ++i)
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
        template <class t_Visitor, typename t_VectorEntryType, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                std::vector<t_VectorEntryType, t_Allocator> &entry,
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
        template <class t_Visitor, typename t_VectorEntryType, class t_Allocator>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_process(
                const t_Visitor &visitor,
                std::vector<t_VectorEntryType, t_Allocator> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            for (std::size_t i = 0; i < entry.size(); ++i)
            {
                apply_process(visitor, entry[i], param);
            }
        }
    }  // namespace process
}  // namespace ariles2
