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
    template <  class t_Reader,
                typename t_First,
                typename t_Second>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                t_Reader & reader,
                std::pair<t_First, t_Second> & entry,
                const typename t_Reader::Parameters & param)
    {
        typename t_Reader::Parameters param_local = param;
        param_local.unset(t_Reader::Parameters::ALLOW_MISSING_ENTRIES);
        reader.template startMap<t_Reader::SIZE_LIMIT_EQUAL>(2);
        readEntry(reader, entry.first, "first", param_local);
        readEntry(reader, entry.second, "second", param_local);
        reader.endMap();
    }


    template <  class t_Reader,
                typename t_Second>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                t_Reader & reader,
                std::pair<std::string, t_Second> & entry,
                const typename t_Reader::Parameters & param)
    {
        if (reader.getBridgeFlags().isSet(BridgeFlags::SLOPPY_PAIRS_SUPPORTED)
                && param.isSet(t_Reader::Parameters::SLOPPY_PAIRS_IF_SUPPORTED))
        {
            std::vector<std::string> entry_names;
            ARILES_ASSERT(true == reader.getMapEntryNames(entry_names), "Could not read names of map entries.");
            ARILES_ASSERT(1 == entry_names.size(), "Wrong number of map entries for a sloppy pair.");
            entry.first = entry_names[0];

            typename t_Reader::Parameters param_local = param;
            param_local.unset(t_Reader::Parameters::ALLOW_MISSING_ENTRIES);
            reader.template startMap<t_Reader::SIZE_LIMIT_EQUAL>(1);

            readEntry(reader, entry.second, entry.first, param_local);

            reader.endMap();
        }
        else
        {
            readBody<t_Reader, std::string, t_Second>(reader, entry, param);
        }
    }



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


    template <  typename t_First,
                typename t_Second>
        bool ARILES_VISIBILITY_ATTRIBUTE
        compare(const std::pair<t_First, t_Second> & left,
                const std::pair<t_First, t_Second> & right,
                const ariles::ComparisonParameters & param)
    {
        ARILES_TRACE_FUNCTION;

        if (false == compare(left.first, right.first, param))
        {
            return (false);
        }

        if (false == compare(left.second, right.second, param))
        {
            return (false);
        }

        return (true);
    }
}



namespace ariles
{
    namespace defaults
    {
        template <  class t_Iterator, 
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE arilesApply(
                    t_Iterator & iterator,
                    std::pair<t_First, t_Second> & entry,
                    const std::string & /*name*/,
                    const typename t_Iterator::DefaultsParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            arilesApply(iterator, entry.first, "", param);
            arilesApply(iterator, entry.second, "", param);
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
            void ARILES_VISIBILITY_ATTRIBUTE arilesApply(
                    t_Iterator & iterator,
                    std::pair<t_First, t_Second> &entry,
                    const std::string & /*name*/,
                    const typename t_Iterator::FinalizeParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            arilesApply(iterator, entry.first, "", param);
            arilesApply(iterator, entry.second, "", param);
        }
    }
}
