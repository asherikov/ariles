/**
    @file
    @author  Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <tuple>
#include <type_traits>
#include <string>
#include "../internal/helpers.h"


namespace ariles2
{
    namespace read
    {
        namespace impl
        {
            template <class t_Visitor, std::size_t... t_Indices, typename... t_Args>
            void apply_read(
                    t_Visitor &visitor,
                    std::tuple<t_Args...> &entry,
                    const typename t_Visitor::Parameters &param,
                    std::index_sequence<t_Indices...>)
            {
                (visitor.visitMapEntry(std::get<t_Indices>(entry), "tuple_" + std::to_string(t_Indices), param, true),
                 ...);
            }
        }  // namespace impl

        template <class t_Visitor, typename... t_Args>
        void apply_read(t_Visitor &visitor, std::tuple<t_Args...> &entry, const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);

            visitor.startMap(visitor.SIZE_LIMIT_EQUAL, sizeof...(t_Args));
            impl::apply_read(visitor, entry, param, std::index_sequence_for<t_Args...>{});
            visitor.endMap();
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        namespace impl
        {
            template <class t_Visitor, std::size_t... t_Indices, typename... t_Args>
            void apply_write(
                    t_Visitor &writer,
                    const std::tuple<t_Args...> &entry,
                    const typename t_Visitor::Parameters &param,
                    std::index_sequence<t_Indices...>)
            {
                (writer.visitMapEntry(std::get<t_Indices>(entry), "tuple_" + std::to_string(t_Indices), param), ...);
            }
        }  // namespace impl

        template <class t_Visitor, typename... t_Args>
        void apply_write(
                t_Visitor &writer,
                const std::tuple<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);

            writer.startMap(param, sizeof...(t_Args));
            impl::apply_write(writer, entry, param, std::index_sequence_for<t_Args...>{});
            writer.endMap();
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        namespace impl
        {
            template <class t_Visitor, std::size_t... t_Indices, typename... t_Args>
            void apply_compare(
                    t_Visitor &visitor,
                    const std::tuple<t_Args...> &left,
                    const std::tuple<t_Args...> &right,
                    const typename t_Visitor::Parameters &param,
                    std::index_sequence<t_Indices...>)
            {
                (apply_compare(visitor, std::get<t_Indices>(left), std::get<t_Indices>(right), param), ...);
            }
        }  // namespace impl

        template <class t_Visitor, typename... t_Args>
        void apply_compare(
                t_Visitor &visitor,
                const std::tuple<t_Args...> &left,
                const std::tuple<t_Args...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);

            impl::apply_compare(visitor, left, right, param, std::index_sequence_for<t_Args...>{});
        }
    }  // namespace compare
}  // namespace ariles2


namespace ariles2
{
    namespace defaults
    {
        namespace impl
        {
            template <class t_Visitor, std::size_t... t_Indices, typename... t_Args>
            void apply_defaults(
                    const t_Visitor &visitor,
                    std::tuple<t_Args...> &entry,
                    const typename t_Visitor::Parameters &param,
                    std::index_sequence<t_Indices...>)
            {
                (apply_defaults(visitor, std::get<t_Indices>(entry), param), ...);
            }
        }  // namespace impl

        template <class t_Visitor, typename... t_Args>
        void apply_defaults(
                const t_Visitor &visitor,
                std::tuple<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);

            impl::apply_defaults(visitor, entry, param, std::index_sequence_for<t_Args...>{});
        }
    }  // namespace defaults
}  // namespace ariles2


namespace ariles2
{
    namespace process
    {
        namespace impl
        {
            template <class t_Visitor, std::size_t... t_Indices, typename... t_Args>
            void apply_process(
                    const t_Visitor &visitor,
                    std::tuple<t_Args...> &entry,
                    const typename t_Visitor::Parameters &param,
                    std::index_sequence<t_Indices...>)
            {
                (apply_process(visitor, std::get<t_Indices>(entry), param), ...);
            }
        }  // namespace impl

        template <class t_Visitor, typename... t_Args>
        void apply_process(
                const t_Visitor &visitor,
                std::tuple<t_Args...> &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);

            impl::apply_process(visitor, entry, param, std::index_sequence_for<t_Args...>{});
        }
    }  // namespace process
}  // namespace ariles2


namespace ariles2
{
    namespace copyfrom
    {
        namespace impl
        {
            template <class t_Visitor, std::size_t... t_Indices, typename... t_Args>
            void apply_copyfrom(
                    t_Visitor &visitor,
                    std::tuple<t_Args...> &left,
                    const std::tuple<t_Args...> &right,
                    const typename t_Visitor::Parameters &param,
                    std::index_sequence<t_Indices...>)
            {
                (apply_copyfrom(visitor, std::get<t_Indices>(left), std::get<t_Indices>(right), param), ...);
            }
        }  // namespace impl

        template <class t_Visitor, typename... t_Args>
        void apply_copyfrom(
                t_Visitor &visitor,
                std::tuple<t_Args...> &left,
                const std::tuple<t_Args...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            impl::apply_copyfrom(visitor, left, right, param, std::index_sequence_for<t_Args...>{});
        }
    }  // namespace copyfrom


    namespace copyto
    {
        namespace impl
        {
            template <class t_Visitor, std::size_t... t_Indices, typename... t_Args>
            void apply_copyto(
                    t_Visitor &visitor,
                    const std::tuple<t_Args...> &left,
                    std::tuple<t_Args...> &right,
                    const typename t_Visitor::Parameters &param,
                    std::index_sequence<t_Indices...>)
            {
                (apply_copyto(visitor, std::get<t_Indices>(left), std::get<t_Indices>(right), param), ...);
            }
        }  // namespace impl

        template <class t_Visitor, typename... t_Args>
        void apply_copyto(
                t_Visitor &visitor,
                const std::tuple<t_Args...> &left,
                std::tuple<t_Args...> &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            impl::apply_copyto(visitor, left, right, param, std::index_sequence_for<t_Args...>{});
        }
    }  // namespace copyto
}  // namespace ariles2
