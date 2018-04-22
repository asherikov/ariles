/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_enum.hpp>
#include <boost/type_traits/is_base_of.hpp>


namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader &reader,
                    boost::shared_ptr<t_Entry> &entry,
                    const bool crash_on_missing_entry)
        {
            ARILES_IGNORE_UNUSED(crash_on_missing_entry);

            bool is_null = true;
            readEntry(reader, is_null, "is_null", true);

            if (true == is_null)
            {
                entry.reset();
            }
            else
            {
                entry = boost::make_shared<t_Entry>();
                readEntry(reader, *entry, "value", true);
            }
        }
    }


    namespace writer
    {
        template <  class t_Writer,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody( t_Writer & writer,
                       const boost::shared_ptr<t_Entry> &entry)
        {
            bool is_null = true;

            if (NULL == entry)
            {
                is_null = true;
                writer.startMap(1);
                writeEntry(writer, is_null, "is_null");
                writer.endMap();
            }
            else
            {
                is_null = false;
                writer.startMap(2);
                writeEntry(writer, is_null, "is_null");
                writeEntry(writer, *entry, "value");
                writer.endMap();
            }
        }
    }
}
