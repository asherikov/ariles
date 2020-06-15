/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <utility>
#include "../internal/helpers.h"
#include "../visitors/serialization.h"

namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, typename t_First, typename t_Second>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                std::pair<t_First, t_Second> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;
            visitor.startMap(t_Visitor::SIZE_LIMIT_EQUAL, 2);

            visitor.override_missing_entries_locally_ = true;
            visitor(entry.first, "first", parameters);

            visitor.override_missing_entries_locally_ = true;
            visitor(entry.second, "second", parameters);
            visitor.endMap();
        }


        template <class t_Visitor, typename t_Second>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                std::pair<std::string, t_Second> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;
            // size = 0 is ok if missing entries are allowed (fallback to standard logic which is going to fail)
            // size > 1 is never ok, due to ambiguity.
            if (true == parameters.sloppy_pairs_ and true == visitor.startIteratedMap(t_Visitor::SIZE_LIMIT_EQUAL, 1))
            {
                ARILES2_ASSERT(
                        true == visitor.startIteratedMapElement(entry.first),
                        "Could not read first element of a sloppy pair.");
                apply_read(visitor, entry.second, parameters);
                visitor.endIteratedMapElement();
                visitor.endIteratedMap();
            }
            else
            {
                apply_read<t_Visitor, std::string, t_Second>(visitor, entry, parameters);
            }
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, typename t_First, typename t_Second>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const std::pair<t_First, t_Second> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            writer.startMap("", 2);
            writer(entry.first, "first", param);
            writer(entry.second, "second", param);
            writer.endMap();
        }



        template <class t_Visitor, typename t_Second>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const std::pair<std::string, t_Second> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == param.sloppy_pairs_ and true == writer.startIteratedMap("", 1))
            {
                writer(entry.second, entry.first, param);
                writer.endIteratedMap();
            }
            else
            {
                // ? Gets mixed up with vector and fails.
                // apply<t_Visitor, std::string, t_Second>(writer, entry, param);
                writer.startMap("", 2);
                writer(entry.first, "first", param);
                writer(entry.second, "second", param);
                writer.endMap();
            }
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, typename t_First, typename t_Second>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const std::pair<t_First, t_Second> &left,
                const std::pair<t_First, t_Second> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            apply_compare(visitor, left.first, right.first, param);
            apply_compare(visitor, left.second, right.second, param);
        }
    }  // namespace compare
}  // namespace ariles2



namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, typename t_First, typename t_Second>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor &visitor,
                std::pair<t_First, t_Second> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            apply_defaults(visitor, entry.first, param);
            apply_defaults(visitor, entry.second, param);
        }
    }  // namespace defaults
}  // namespace ariles2



namespace ariles2
{
    namespace process
    {
        template <class t_Visitor, typename t_First, typename t_Second>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_process(
                const t_Visitor &visitor,
                std::pair<t_First, t_Second> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            apply_process(visitor, entry.first, param);
            apply_process(visitor, entry.second, param);
        }
    }  // namespace process
}  // namespace ariles2
