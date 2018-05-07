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
                    const bool /*crash_on_missing_entry*/)
        {
            std::size_t size = reader.startArray();
            entry.clear();
            for(std::size_t i = 0; i < size; ++i)
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


#ifdef ARILES_ENABLE_SLOPPY_MAP
    namespace reader
    {
        template <  class t_Reader,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::map<std::string, t_Value, t_Compare, t_Allocator> & entry,
                    const bool /*crash_on_missing_entry*/,
                    ARILES_IS_CHILD_ENABLER_TYPE(ariles::SloppyMapReaderBase, t_Reader) * /*dummy*/)
        {
            std::vector<std::string> entry_names;
            ARILES_ASSERT(true == reader.getEntryNames(entry_names), "Could not read names of map entries.");
            entry.clear();
            for (std::size_t i = 0; i < entry_names.size(); ++i)
            {
                t_Value entry_value;
                readEntry(reader, entry_value, entry_names[i], true);
                entry[entry_names[i]] = entry_value;
            }
        }
    }


    namespace writer
    {
        template <  class t_Writer,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                    t_Writer & writer,
                    const std::map<std::string, t_Value, t_Compare, t_Allocator> & entry,
                    ARILES_IS_CHILD_ENABLER_TYPE(ariles::SloppyMapWriterBase, t_Writer) * /*dummy*/)
        {
            writer.startMap(entry.size());
            for (
                typename std::map<std::string, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                it != entry.end();
                ++it)
            {
                writeEntry(writer, it->second, it->first);
            }
            writer.endMap();
        }
    }
#endif
}
