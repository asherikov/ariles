/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <vector>

namespace ariles
{
    template <  class t_Reader,
                typename t_VectorEntryType,
                class t_Allocator,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                t_Reader & reader,
                std::vector<t_VectorEntryType, t_Allocator> & entry,
                const t_Flags & param)
    {
        entry.resize(reader.startArray());
        for(std::size_t i = 0; i < entry.size(); ++i)
        {
            readBody(reader, entry[i], param);
            reader.shiftArray();
        }
        reader.endArray();
    }



    template <  class t_Writer,
                typename t_VectorEntryType,
                class t_Allocator,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                t_Writer & writer,
                const std::vector<t_VectorEntryType, t_Allocator> & entry,
                const t_Flags & param)
    {
        writer.startArray(entry.size(), param.isSet(ConfigurableFlags::COMPACT_ARRAYS_IF_SUPPORTED));
        for (std::size_t i = 0; i < entry.size(); ++i)
        {
            writeBody(writer, entry[i], param);
            writer.shiftArray();
        }
        writer.endArray();
    }



    template <  typename t_VectorEntryType,
                class t_Allocator,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(
                std::vector<t_VectorEntryType, t_Allocator> & entry,
                const t_Flags & /*param*/)
    {
        ARILES_TRACE_FUNCTION;
        entry.clear();
    }


    template <  typename t_VectorEntryType,
                class t_Allocator>
        void ARILES_VISIBILITY_ATTRIBUTE
        finalize(   std::vector<t_VectorEntryType, t_Allocator> & entry,
                    const ArilesNamespaceLookupTrigger &trigger)
    {
        ARILES_TRACE_FUNCTION;
        for (std::size_t i = 0; i < entry.size(); ++i)
        {
            finalize(entry[i], trigger);
        }
    }


    template <  typename t_VectorEntryType,
                class t_Allocator>
        bool ARILES_VISIBILITY_ATTRIBUTE
        compare(const std::vector<t_VectorEntryType, t_Allocator> &left,
                const std::vector<t_VectorEntryType, t_Allocator> &right,
                const ariles::ComparisonParameters & param)
    {
        ARILES_TRACE_FUNCTION;

        if (left.size() != right.size())
        {
            return (false);
        }

        for (std::size_t i = 0; i < left.size(); ++i)
        {
            if (false == compare(left[i], right[i], param))
            {
                return (false);
            }
        }

        return (true);
    }
}
