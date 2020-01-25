/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <utility>

namespace ariles
{
    namespace read
    {
        template <  class t_Iterator,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator & iterator,
                    std::pair<t_First, t_Second> & entry,
                    const typename t_Iterator::ReadParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            typename t_Iterator::ReadParameters param_local = param;
            param_local.unset(t_Iterator::ReadParameters::ALLOW_MISSING_ENTRIES);
            iterator.template startMap<t_Iterator::SIZE_LIMIT_EQUAL>(2);
            arilesEntryApply(iterator, entry.first, "first", param_local);
            arilesEntryApply(iterator, entry.second, "second", param_local);
            iterator.endMap();
        }


        template <  class t_Iterator,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator & iterator,
                    std::pair<std::string, t_Second> & entry,
                    const typename t_Iterator::ReadParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (iterator.getBridgeFlags().isSet(BridgeFlags::SLOPPY_PAIRS_SUPPORTED)
                    && param.isSet(t_Iterator::ReadParameters::SLOPPY_PAIRS_IF_SUPPORTED))
            {
                std::vector<std::string> entry_names;
                ARILES_ASSERT(true == iterator.getMapEntryNames(entry_names), "Could not read names of map entries.");
                ARILES_ASSERT(1 == entry_names.size(), "Wrong number of map entries for a sloppy pair.");
                entry.first = entry_names[0];

                typename t_Iterator::ReadParameters param_local = param;
                param_local.unset(t_Iterator::ReadParameters::ALLOW_MISSING_ENTRIES);
                iterator.template startMap<t_Iterator::SIZE_LIMIT_EQUAL>(1);

                arilesEntryApply(iterator, entry.second, entry.first, param_local);

                iterator.endMap();
            }
            else
            {
                apply<t_Iterator, std::string, t_Second>(iterator, entry, param);
            }
        }
    }
}


namespace ariles
{
    template <  class t_Writer,
                typename t_First,
                typename t_Second>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                t_Writer & writer,
                const std::pair<t_First, t_Second> & entry,
                const typename t_Writer::Parameters & param)
    {
        writer.startMap(2);
        writeEntry(writer, entry.first, "first", param);
        writeEntry(writer, entry.second, "second", param);
        writer.endMap();
    }



    template <  class t_Writer,
                typename t_Second>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                t_Writer & writer,
                const std::pair<std::string, t_Second> & entry,
                const typename t_Writer::Parameters & param)
    {
        if (writer.getBridgeFlags().isSet(BridgeFlags::SLOPPY_PAIRS_SUPPORTED)
                && param.isSet(t_Writer::Parameters::SLOPPY_PAIRS_IF_SUPPORTED))
        {
            writer.startMap(1);
            writeEntry(writer, entry.second, entry.first, param);
            writer.endMap();
        }
        else
        {
            // ? Gets mixed up with vector and fails.
            // writeBody<t_Writer, std::string, t_Second>(writer, entry, param);
            writer.startMap(2);
            writeEntry(writer, entry.first, "first", param);
            writeEntry(writer, entry.second, "second", param);
            writer.endMap();
        }
    }
}


namespace ariles
{
    namespace compare
    {
        template <  class t_Iterator,
                    typename t_First,
                    typename t_Second>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    const std::pair<t_First, t_Second> & left,
                    const std::pair<t_First, t_Second> & right,
                    const typename t_Iterator::CompareParameters & param)
        {
            ARILES_TRACE_FUNCTION;

            if (false == apply(iterator, left.first, right.first, param))
            {
                return (false);
            }

            if (false == apply(iterator, left.second, right.second, param))
            {
                return (false);
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
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    std::pair<t_First, t_Second> & entry,
                    const typename t_Iterator::DefaultsParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            apply(iterator, entry.first, param);
            apply(iterator, entry.second, param);
        }
    }
}



namespace ariles
{
    namespace finalize
    {
        template <  class t_Iterator,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    std::pair<t_First, t_Second> &entry,
                    const typename t_Iterator::FinalizeParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            apply(iterator, entry.first, param);
            apply(iterator, entry.second, param);
        }
    }
}
