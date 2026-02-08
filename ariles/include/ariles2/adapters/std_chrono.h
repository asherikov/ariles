/**
    @file
    @author  Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <chrono>
#include <ratio>
#include "../internal/helpers.h"


namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, class t_Rep, class t_Period>
        void apply_read(
                t_Visitor &visitor,
                std::chrono::duration<t_Rep, t_Period> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            t_Rep value;
            visitor.readElement(value);
            entry = std::chrono::duration<t_Rep, t_Period>(value);
        }


        template <class t_Visitor, class t_Clock, class t_Duration>
        void apply_read(
                t_Visitor &visitor,
                std::chrono::time_point<t_Clock, t_Duration> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            typename t_Duration::rep value;
            visitor.readElement(value);
            entry = std::chrono::time_point<t_Clock, t_Duration>(std::chrono::duration_cast<t_Duration>(
                    std::chrono::duration<typename t_Duration::rep, typename t_Duration::period>(value)));
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, class t_Rep, class t_Period>
        void apply_write(
                t_Visitor &writer,
                const std::chrono::duration<t_Rep, t_Period> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            t_Rep count = entry.count();
            writer.writeElement(count, param);
        }


        template <class t_Visitor, class t_Clock, class t_Duration>
        void apply_write(
                t_Visitor &writer,
                const std::chrono::time_point<t_Clock, t_Duration> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            typename t_Duration::rep count = std::chrono::duration_cast<t_Duration>(entry.time_since_epoch()).count();
            writer.writeElement(count, param);
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, class t_Rep, class t_Period>
        void apply_compare(
                t_Visitor &visitor,
                const std::chrono::duration<t_Rep, t_Period> &left,
                const std::chrono::duration<t_Rep, t_Period> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            t_Rep left_count = left.count();
            t_Rep right_count = right.count();
            apply_compare(visitor, left_count, right_count, param);
        }


        template <class t_Visitor, class t_Clock, class t_Duration>
        void apply_compare(
                t_Visitor &visitor,
                const std::chrono::time_point<t_Clock, t_Duration> &left,
                const std::chrono::time_point<t_Clock, t_Duration> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            typename t_Duration::rep left_count = left.time_since_epoch().count();
            typename t_Duration::rep right_count = right.time_since_epoch().count();
            apply_compare(visitor, left_count, right_count, param);
        }
    }  // namespace compare
}  // namespace ariles2


namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, class t_Rep, class t_Period>
        void apply_defaults(
                const t_Visitor & /*visitor*/,
                std::chrono::duration<t_Rep, t_Period> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            entry = std::chrono::duration<t_Rep, t_Period>::zero();
        }


        template <class t_Visitor, class t_Clock, class t_Duration>
        void apply_defaults(
                const t_Visitor & /*visitor*/,
                std::chrono::time_point<t_Clock, t_Duration> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            entry = std::chrono::time_point<t_Clock, t_Duration>::min();
        }
    }  // namespace defaults
}  // namespace ariles2


namespace ariles2
{
    namespace process
    {
        template <class t_Visitor, class t_Rep, class t_Period>
        void apply_process(
                const t_Visitor &visitor,
                std::chrono::duration<t_Rep, t_Period> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            t_Rep count = entry.count();
            apply_process(visitor, count, param);
            entry = std::chrono::duration<t_Rep, t_Period>(count);
        }


        template <class t_Visitor, class t_Clock, class t_Duration>
        void apply_process(
                const t_Visitor &visitor,
                std::chrono::time_point<t_Clock, t_Duration> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            t_Duration duration = entry.time_since_epoch();
            apply_process(visitor, duration, param);
            entry = std::chrono::time_point<t_Clock, t_Duration>(duration);
        }
    }  // namespace process
}  // namespace ariles2


namespace ariles2
{
    namespace copyfrom
    {
        template <class t_Visitor, class t_Rep, class t_Period>
        void apply_copyfrom(
                t_Visitor & /*visitor*/,
                std::chrono::duration<t_Rep, t_Period> &left,
                const std::chrono::duration<t_Rep, t_Period> &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            t_Rep right_count = right.count();
            std::chrono::duration<t_Rep, t_Period> converted_right =
                    std::chrono::duration_cast<std::chrono::duration<t_Rep, t_Period>>(
                            std::chrono::duration<t_Rep, t_Period>(right_count));
            left = converted_right;
        }


        template <class t_Visitor, class t_Clock, class t_Duration>
        void apply_copyfrom(
                t_Visitor & /*visitor*/,
                std::chrono::time_point<t_Clock, t_Duration> &left,
                const std::chrono::time_point<t_Clock, t_Duration> &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            t_Duration right_duration = right.time_since_epoch();
            t_Duration converted_duration = std::chrono::duration_cast<t_Duration>(right_duration);
            left = std::chrono::time_point<t_Clock, t_Duration>(converted_duration);
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <class t_Visitor, class t_Rep, class t_Period>
        void apply_copyto(
                t_Visitor & /*visitor*/,
                const std::chrono::duration<t_Rep, t_Period> &left,
                std::chrono::duration<t_Rep, t_Period> &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            t_Rep left_count = left.count();
            t_Rep converted_left = std::chrono::duration_cast<std::chrono::duration<t_Rep, t_Period>>(
                                           std::chrono::duration<t_Rep, t_Period>(left_count))
                                           .count();
            right = std::chrono::duration<t_Rep, t_Period>(converted_left);
        }


        template <class t_Visitor, class t_Clock, class t_Duration>
        void apply_copyto(
                t_Visitor & /*visitor*/,
                const std::chrono::time_point<t_Clock, t_Duration> &left,
                std::chrono::time_point<t_Clock, t_Duration> &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            t_Duration left_duration = left.time_since_epoch();
            t_Duration converted_duration = std::chrono::duration_cast<t_Duration>(left_duration);
            right = std::chrono::time_point<t_Clock, t_Duration>(converted_duration);
        }
    }  // namespace copyto
}  // namespace ariles2
