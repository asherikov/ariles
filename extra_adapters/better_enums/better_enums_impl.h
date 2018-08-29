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
    namespace reader
    {
        template <  class t_Reader,
                    class t_BetterEnum>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader &reader,
                        t_BetterEnum &entry,
                        const ariles::ConfigurableFlags & /*param*/,
                        const typename t_BetterEnum::_integral * /*dummy*/,
                        const typename t_BetterEnum::_value_iterable * /*dummy*/,
                        const typename t_BetterEnum::_name_iterable * /*dummy*/,
                        const typename t_BetterEnum::_value_iterator * /*dummy*/,
                        const typename t_BetterEnum::_name_iterator * /*dummy*/)
        {
            std::string enum_value;
            reader.readElement(enum_value);
            entry = t_BetterEnum::_from_string(enum_value.c_str());
        }
    }


    namespace writer
    {
        template <  class t_Writer,
                    class t_BetterEnum>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const t_BetterEnum &entry,
                        const ariles::ConfigurableFlags & /*param*/,
                        const typename t_BetterEnum::_integral * /*dummy*/,
                        const typename t_BetterEnum::_value_iterable * /*dummy*/,
                        const typename t_BetterEnum::_name_iterable * /*dummy*/,
                        const typename t_BetterEnum::_value_iterator * /*dummy*/,
                        const typename t_BetterEnum::_name_iterator * /*dummy*/)
        {
            writer.writeElement(std::string(entry._to_string()));
        }
    }
}
