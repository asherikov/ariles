/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    namespace read
    {
        template <  class t_Visitor,
                    class t_BetterEnum,
                    class t_Flags>
            void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                    t_Visitor &visitor,
                    t_BetterEnum &entry,
                    const t_Flags & /*param*/,
                    const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES_TRACE_FUNCTION;
            std::string enum_value;
            visitor.readElement(enum_value);
            entry = t_BetterEnum::_from_string(enum_value.c_str());
        }
    }
}


namespace ariles
{
    namespace write
    {
        template <  class t_Visitor,
                    class t_BetterEnum,
                    class t_Flags>
            void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                    t_Visitor & writer,
                    const t_BetterEnum &entry,
                    const t_Flags & /*param*/,
                    const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES_TRACE_FUNCTION;
            writer.writeElement(std::string(entry._to_string()));
        }
    }
}


namespace ariles
{
    namespace compare
    {
        template <class t_Visitor, class t_BetterEnum>
            bool ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                    const t_Visitor & /*visitor*/,
                    const t_BetterEnum &left,
                    const t_BetterEnum &right,
                    const typename t_Visitor::Parameters & /*param*/,
                    const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES_TRACE_FUNCTION;
            return (left == right);
        }
    }
}



namespace ariles
{
    namespace defaults
    {
        template <  class t_Visitor,
                    class t_BetterEnum>
            void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                    t_Visitor & /*visitor*/,
                    t_BetterEnum &entry,
                    const typename t_Visitor::Parameters & /*param*/,
                    const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES_TRACE_FUNCTION;
            entry = t_BetterEnum::_from_integral_unchecked(0);
        }
    }
}



namespace ariles
{
    namespace finalize
    {
        template <  class t_Visitor,
                    class t_BetterEnum>
            void ARILES_VISIBILITY_ATTRIBUTE apply_finalize(
                        t_Visitor & /*visitor*/,
                        t_BetterEnum & /*entry*/,
                        const typename t_Visitor::Parameters & /*param*/,
                        const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
        {
            ARILES_TRACE_FUNCTION;
        }
    }
}
