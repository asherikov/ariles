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
    template <  class t_Reader,
                class t_BetterEnum,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE
        readBody(   t_Reader &reader,
                    t_BetterEnum &entry,
                    const t_Flags & /*param*/,
                    const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
    {
        std::string enum_value;
        reader.readElement(enum_value);
        entry = t_BetterEnum::_from_string(enum_value.c_str());
    }


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


    template <  class t_BetterEnum,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE
        setDefaults(t_BetterEnum &entry,
                    const t_Flags & /*param*/,
                    const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
    {
        ARILES_TRACE_FUNCTION;
        entry = t_BetterEnum::_from_integral_unchecked(0);
    }


    template <class t_BetterEnum>
        void ARILES_VISIBILITY_ATTRIBUTE
        finalize(   t_BetterEnum & /*entry*/,
                    const ArilesNamespaceLookupTrigger &,
                    const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                    const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL)
    {
        ARILES_TRACE_FUNCTION;
    }
}
