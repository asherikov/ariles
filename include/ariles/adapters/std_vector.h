/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_ADAPTER_STD_VECTOR

#include <vector>

namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_VectorEntryType>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(   t_Reader & reader,
                                    std::vector<t_VectorEntryType> & entry,
                                    const bool crash_on_missing_entry = false);
    }


    namespace writer
    {
        /**
         * @brief Read configuration entry (std::vector)
         *
         * @tparam t_VectorEntryType type of the entry of std::vector
         *
         * @param[in] entry      data
         * @param[in] entry_name name
         */
        template <  class t_Writer,
                    typename t_VectorEntryType>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody( t_Writer & writer,
                            const std::vector<t_VectorEntryType> & entry);
    }
}
