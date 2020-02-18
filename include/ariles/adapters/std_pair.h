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
        template <  class t_Visitor,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                    t_Visitor & visitor,
                    std::pair<t_First, t_Second> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            typename t_Visitor::Parameters param_local = param;
            param_local.unset(t_Visitor::Parameters::ALLOW_MISSING_ENTRIES);
            visitor.template startMap<t_Visitor::SIZE_LIMIT_EQUAL>(2);
            visitor(entry.first, "first", param_local);
            visitor(entry.second, "second", param_local);
            visitor.endMap();
        }


        template <  class t_Visitor,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                    t_Visitor & visitor,
                    std::pair<std::string, t_Second> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (visitor.getBridgeFlags().isSet(BridgeFlags::SLOPPY_PAIRS_SUPPORTED)
                    && param.isSet(t_Visitor::Parameters::SLOPPY_PAIRS_IF_SUPPORTED))
            {
                std::vector<std::string> entry_names;
                ARILES_ASSERT(true == visitor.getMapEntryNames(entry_names), "Could not read names of map entries.");
                ARILES_ASSERT(1 == entry_names.size(), "Wrong number of map entries for a sloppy pair.");
                entry.first = entry_names[0];

                typename t_Visitor::Parameters param_local = param;
                param_local.unset(t_Visitor::Parameters::ALLOW_MISSING_ENTRIES);
                visitor.template startMap<t_Visitor::SIZE_LIMIT_EQUAL>(1);

                visitor(entry.second, entry.first, param_local);

                visitor.endMap();
            }
            else
            {
                apply_read<t_Visitor, std::string, t_Second>(visitor, entry, param);
            }
        }
    }
}


namespace ariles
{
    namespace write
    {
        template <  class t_Visitor,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                    t_Visitor & writer,
                    const std::pair<t_First, t_Second> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            writer.startMap(2);
            writer(entry.first, "first", param);
            writer(entry.second, "second", param);
            writer.endMap();
        }



        template <  class t_Visitor,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                    t_Visitor & writer,
                    const std::pair<std::string, t_Second> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (writer.getBridgeFlags().isSet(BridgeFlags::SLOPPY_PAIRS_SUPPORTED)
                    && param.isSet(t_Visitor::Parameters::SLOPPY_PAIRS_IF_SUPPORTED))
            {
                writer.startMap(1);
                writer(entry.second, entry.first, param);
                writer.endMap();
            }
            else
            {
                // ? Gets mixed up with vector and fails.
                // apply<t_Visitor, std::string, t_Second>(writer, entry, param);
                writer.startMap(2);
                writer(entry.first, "first", param);
                writer(entry.second, "second", param);
                writer.endMap();
            }
        }
    }
}


namespace ariles
{
    namespace compare
    {
        template <  class t_Visitor,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                    t_Visitor & visitor,
                    const std::pair<t_First, t_Second> & left,
                    const std::pair<t_First, t_Second> & right,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;

            apply_compare(visitor, left.first, right.first, param);
            apply_compare(visitor, left.second, right.second, param);
        }
    }
}



namespace ariles
{
    namespace defaults
    {
        template <  class t_Visitor,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                    const t_Visitor & visitor,
                    std::pair<t_First, t_Second> & entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            apply_defaults(visitor, entry.first, param);
            apply_defaults(visitor, entry.second, param);
        }
    }
}



namespace ariles
{
    namespace finalize
    {
        template <  class t_Visitor,
                    typename t_First,
                    typename t_Second>
            void ARILES_VISIBILITY_ATTRIBUTE apply_finalize(
                    const t_Visitor & visitor,
                    std::pair<t_First, t_Second> &entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            apply_finalize(visitor, entry.first, param);
            apply_finalize(visitor, entry.second, param);
        }
    }
}
