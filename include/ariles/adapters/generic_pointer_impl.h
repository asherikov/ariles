/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

// ARILES_POINTER_TYPE(entry_type)
// ARILES_POINTER_ALLOCATE(entry_type, pointer)
// ARILES_POINTER_RESET(pointer)

namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader &reader,
                    ARILES_POINTER_TYPE(t_Entry) &entry,
                    const bool /*crash_on_missing_entry*/)
        {
            bool is_null = true;
            readEntry(reader, is_null, "is_null", true);

            if (true == is_null)
            {
                ARILES_POINTER_RESET(entry);
            }
            else
            {
                ARILES_POINTER_ALLOCATE(t_Entry, entry);
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
                       const ARILES_POINTER_TYPE(t_Entry) &entry)
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

#undef ARILES_POINTER_TYPE
#undef ARILES_POINTER_ALLOCATE
#undef ARILES_POINTER_RESET
