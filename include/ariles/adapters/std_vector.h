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
    namespace read
    {
        template <  class t_Iterator,
                    typename t_VectorEntryType,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator & iterator,
                    std::vector<t_VectorEntryType, t_Allocator> & entry,
                    const typename t_Iterator::ReadParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            entry.resize(iterator.startArray());
            for(std::size_t i = 0; i < entry.size(); ++i)
            {
                apply(iterator, entry[i], param);
                iterator.shiftArray();
            }
            iterator.endArray();
        }
    }
}


namespace ariles
{
    template <  class t_Writer,
                typename t_VectorEntryType,
                class t_Allocator>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                t_Writer & writer,
                const std::vector<t_VectorEntryType, t_Allocator> & entry,
                const typename t_Writer::Parameters & param)
    {
        writer.startArray(entry.size(), param.isSet(t_Writer::Parameters::COMPACT_ARRAYS_IF_SUPPORTED));
        for (std::size_t i = 0; i < entry.size(); ++i)
        {
            writeBody(writer, entry[i], param);
            writer.shiftArray();
        }
        writer.endArray();
    }
}


namespace ariles
{
    namespace compare
    {
        template <  class t_Iterator,
                    typename t_VectorEntryType,
                    class t_Allocator>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    const std::vector<t_VectorEntryType, t_Allocator> &left,
                    const std::vector<t_VectorEntryType, t_Allocator> &right,
                    const typename t_Iterator::CompareParameters & param)
        {
            ARILES_TRACE_FUNCTION;

            if (left.size() != right.size())
            {
                return (false);
            }

            for (std::size_t i = 0; i < left.size(); ++i)
            {
                if (false == apply(iterator, left[i], right[i], param))
                {
                    return (false);
                }
            }

            return (true);
        }
    }
}


namespace ariles
{
    namespace defaults
    {
        template <  class t_Iterator,
                    typename t_VectorEntryType,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & /*iterator*/,
                    std::vector<t_VectorEntryType, t_Allocator> & entry,
                    const typename t_Iterator::DefaultsParameters & /*param*/)
        {
            ARILES_TRACE_FUNCTION;
            entry.clear();
        }
    }
}



namespace ariles
{
    namespace finalize
    {
        template <  class t_Iterator,
                    typename t_VectorEntryType,
                    class t_Allocator>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    std::vector<t_VectorEntryType, t_Allocator> & entry,
                    const typename t_Iterator::FinalizeParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            for (std::size_t i = 0; i < entry.size(); ++i)
            {
                apply(iterator, entry[i], param);
            }
        }
    }
}
