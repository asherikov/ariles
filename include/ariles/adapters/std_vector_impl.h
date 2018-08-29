/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_VectorEntryType,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader & reader,
                    std::vector<t_VectorEntryType, t_Allocator> & entry,
                    const ariles::ConfigurableFlags & param)
        {
            entry.resize(reader.startArray());
            for(std::size_t i = 0; i < entry.size(); ++i)
            {
                readBody(reader, entry[i], param);
                reader.shiftArray();
            }
            reader.endArray();
        }
    }


    namespace writer
    {
        template <  class t_Writer,
                    typename t_VectorEntryType,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                    t_Writer & writer,
                    const std::vector<t_VectorEntryType, t_Allocator> & entry,
                    const ariles::ConfigurableFlags & param)
        {
            writer.startArray(entry.size(), param.isSet(ConfigurableFlags::COMPACT_ARRAYS_IF_SUPPORTED));
            for (std::size_t i = 0; i < entry.size(); ++i)
            {
                writeBody(writer, entry[i], param);
                writer.shiftArray();
            }
            writer.endArray();
        }
    }
}
