/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_INCLUDED_ADAPTER_BETTER_ENUMS


namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    class t_BetterEnum>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader &reader,
                        t_BetterEnum &entry,
                        const ariles::ConfigurableParameters & param,
                        const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL);
    }


    namespace writer
    {
        /**
         * @brief Write a configuration entry (vector)
         *
         * @tparam t_Derived Eigen template parameter
         *
         * @param[in] entry      data
         * @param[in] entry_name name
         */
        template <  class t_Writer,
                    class t_BetterEnum>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const t_BetterEnum &entry,
                        const ariles::ConfigurableParameters & param,
                        const typename t_BetterEnum::_integral * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_value_iterable * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_name_iterable * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_value_iterator * /*dummy*/ = NULL,
                        const typename t_BetterEnum::_name_iterator * /*dummy*/ = NULL);
    }
}
