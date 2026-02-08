/**
    @file
    @author  Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <filesystem>
#include <string>
#include "../internal/helpers.h"

namespace ariles2
{
    namespace read
    {
        template <class t_Visitor>
        void apply_read(t_Visitor &visitor, std::filesystem::path &entry, const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            std::string path_str;
            visitor.readElement(path_str);
            entry = std::filesystem::path(path_str);
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor>
        void apply_write(
                t_Visitor &writer,
                const std::filesystem::path &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_UNUSED_ARG(param);
            writer.writeElement(entry.string(), param);
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor>
        void apply_compare(
                t_Visitor &visitor,
                const std::filesystem::path &left,
                const std::filesystem::path &right,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            apply_compare(visitor, left.string(), right.string(), param);
        }
    }  // namespace compare
}  // namespace ariles2


namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor>
        void apply_defaults(
                const t_Visitor & /*visitor*/,
                std::filesystem::path &entry,
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
        template <class t_Visitor>
        void apply_process(
                const t_Visitor &visitor,
                std::filesystem::path &entry,
                const typename t_Visitor::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            std::string path_str = entry.string();
            apply_process(visitor, path_str, param);
            entry = std::filesystem::path(path_str);
        }
    }  // namespace process
}  // namespace ariles2


namespace ariles2
{
    namespace copyfrom
    {
        template <class t_Visitor>
        void apply_copyfrom(
                t_Visitor & /*visitor*/,
                std::filesystem::path &left,
                const std::filesystem::path &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            left = right;
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <class t_Visitor>
        void apply_copyto(
                t_Visitor & /*visitor*/,
                const std::filesystem::path &left,
                std::filesystem::path &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            CPPUT_TRACE_FUNCTION;
            right = left;
        }
    }  // namespace copyto
}  // namespace ariles2
