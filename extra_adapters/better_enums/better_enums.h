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
        template <  class t_Iterator,
                    class t_BetterEnum,
                    class t_Flags>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator &iterator,
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
            iterator.readElement(enum_value);
            entry = t_BetterEnum::_from_string(enum_value.c_str());
        }
    }
}


namespace ariles
{
    template <  class t_Writer,
                class t_BetterEnum,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE
        writeBody(  t_Writer & writer,
                    const t_BetterEnum &entry,
                    const t_Flags & /*param*/,
                    const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
    {
        writer.writeElement(std::string(entry._to_string()));
    }
}


namespace ariles
{
    namespace compare
    {
        template <class t_Iterator, class t_BetterEnum>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & /*iterator*/,
                    const t_BetterEnum &left,
                    const t_BetterEnum &right,
                    const typename t_Iterator::CompareParameters & /*param*/,
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
        template <  class t_Iterator,
                    class t_BetterEnum>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator & /*iterator*/,
                    t_BetterEnum &entry,
                    const typename t_Iterator::DefaultsParameters & /*param*/,
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
        template <  class t_Iterator,
                    class t_BetterEnum>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                        t_Iterator & /*iterator*/,
                        t_BetterEnum & /*entry*/,
                        const typename t_Iterator::FinalizeParameters & /*param*/,
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
