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
         */
        template <  class t_Reader,
                    typename t_Key,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const ariles::ConfigurableParameters & param)
        {
            std::size_t size = reader.startArray();
            entry.clear();
            ariles::ConfigurableParameters param_local = param;
            param_local.crash_on_missing_entry_ = true;
            for(std::size_t i = 0; i < size; ++i)
            {
                std::pair<t_Key, t_Value> map_entry;

                readBody(reader, map_entry, param_local);

                entry.insert(map_entry);

                reader.shiftArray();
            }
            reader.endArray();
        }


        template <  class t_Reader,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::map<std::string, t_Value, t_Compare, t_Allocator> & entry,
                    const ariles::ConfigurableParameters & param,
                    typename t_Reader::SloppyMapReaderIndicatorType * /*dummy*/)
        {
            if (true == param.enable_sloppy_maps_if_supported_)
            {
                std::vector<std::string> entry_names;
                ARILES_ASSERT(true == reader.getEntryNames(entry_names), "Could not read names of map entries.");
                entry.clear();
                ariles::ConfigurableParameters param_local = param;
                param_local.crash_on_missing_entry_ = true;
                reader.startMap();
                for (std::size_t i = 0; i < entry_names.size(); ++i)
                {
                    t_Value entry_value;
                    readEntry(reader, entry_value, entry_names[i], param_local);
                    entry[entry_names[i]] = entry_value;
                }
                reader.endMap();
            }
            else
            {
                readBody<t_Reader, std::string, t_Value, t_Compare, t_Allocator>(reader, entry, param);
            }
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
                    const std::map<t_Key, t_Value, t_Compare, t_Allocator> & entry,
                    const ariles::ConfigurableParameters & param)
        {
            writer.startArray(entry.size(), param.compact_arrays_if_supported_);
            for (
                typename std::map<t_Key, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                it != entry.end();
                ++it)
            {
                writeBody(writer, *it, param);
                writer.shiftArray();
            }
            writer.endArray();
        }


        template <  class t_Writer,
                    typename t_Value,
                    class t_Compare,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                    t_Writer & writer,
                    const std::map<std::string, t_Value, t_Compare, t_Allocator> & entry,
                    const ariles::ConfigurableParameters & param,
                    typename t_Writer::SloppyMapWriterIndicatorType * /*dummy*/)
        {
            if (true == param.enable_sloppy_maps_if_supported_)
            {
                writer.startMap(entry.size());
                for (
                    typename std::map<std::string, t_Value, t_Compare, t_Allocator>::const_iterator it = entry.begin();
                    it != entry.end();
                    ++it)
                {
                    writeEntry(writer, it->second, it->first, param);
                }
                writer.endMap();
            }
            else
            {
                writeBody<t_Writer, std::string, t_Value, t_Compare, t_Allocator>(writer, entry, param);
            }
        }
    }
}
