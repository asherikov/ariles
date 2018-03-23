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
        /**
         * @brief Read configuration entry (std::map)
         *
         * @tparam t_VectorEntryType type of the entry of std::vector
         *
         * @param[out] entry      configuration parameter
         * @param[in]  crash_on_missing_entry
         */
        template <  class t_Reader,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const bool crash_on_missing_entry)
        {
            ARILES_IGNORE_UNUSED(crash_on_missing_entry);

            reader.startArray();
            entry.clear();
            for(std::size_t i = 0; i < entry.size(); ++i)
            {
                std::pair<t_Key, t_Value> map_entry;

                readBody(reader, map_entry, true);

                entry.insert(map_entry);

                reader.shiftArray();
            }
            reader.endArray();
        }
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
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                    t_Writer & writer,
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry)
        {
            writer.startArray(entry.size());
            for (
                typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                it != entry.end();
                ++it)
            {
                writeBody(writer, *it);
                writer.shiftArray();
            }
            writer.endArray();
        }
    }
}
