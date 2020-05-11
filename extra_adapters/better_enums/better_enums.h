/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "../internal/helpers.h"

namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, class t_BetterEnum, class t_Flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                t_BetterEnum &entry,
                const t_Flags & /*param*/,
                const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES2_TRACE_FUNCTION;
            std::string enum_value;
            visitor.readElement(enum_value);
            entry = t_BetterEnum::_from_string(enum_value.c_str());
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, class t_BetterEnum, class t_Flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const t_BetterEnum &entry,
                const t_Flags & /*param*/,
                const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES2_TRACE_FUNCTION;
            writer.writeElement(std::string(entry._to_string()));
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, class t_BetterEnum>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const t_BetterEnum &left,
                const t_BetterEnum &right,
                const typename t_Visitor::Parameters & /*param*/,
                const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES2_TRACE_FUNCTION;
            visitor.equal_ &= (left == right);
        }
    }  // namespace compare
}  // namespace ariles2



namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, class t_BetterEnum>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                t_Visitor & /*visitor*/,
                t_BetterEnum &entry,
                const typename t_Visitor::Parameters & /*param*/,
                const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES2_TRACE_FUNCTION;
            if (t_BetterEnum::_size() > 0)
            {
                entry = t_BetterEnum::_values()[0];
            }
        }
    }  // namespace defaults
}  // namespace ariles2
