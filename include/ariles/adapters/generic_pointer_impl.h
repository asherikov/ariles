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
// ARILES_POINTER_CHECK_DEFINED(pointer)

namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader &reader,
                    ARILES_POINTER_TYPE(t_Entry) &entry,
                    const ariles::ConfigurableParameters & param)
        {
            bool is_null = true;

            ariles::ConfigurableParameters param_local = param;
            param_local.crash_on_missing_entry_ = true;

            const std::size_t num_entries = reader.startMap();
            ARILES_ASSERT((1 == num_entries) || (2 == num_entries), "Wrong number of entries.");
            readEntry(reader, is_null, "is_null", param_local);

            if (true == is_null)
            {
                ARILES_POINTER_RESET(entry);
            }
            else
            {
                ARILES_ASSERT(2 == num_entries, "Wrong number of entries.");
                ARILES_POINTER_ALLOCATE(t_Entry, entry);
                readEntry(reader, *entry, "value", param_local);
            }
            reader.endMap();
        }
    }


    namespace writer
    {
        template <  class t_Writer,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody( t_Writer & writer,
                       const ARILES_POINTER_TYPE(t_Entry) &entry,
                       const ariles::ConfigurableParameters & param)
        {
            bool is_null = true;

            if (ARILES_POINTER_CHECK_DEFINED(entry))
            {
                is_null = true;
                writer.startMap(1);
                writeEntry(writer, is_null, "is_null", param);
                writer.endMap();
            }
            else
            {
                is_null = false;
                writer.startMap(2);
                writeEntry(writer, is_null, "is_null", param);
                writeEntry(writer, *entry, "value", param);
                writer.endMap();
            }
        }
    }
}

#undef ARILES_POINTER_TYPE
#undef ARILES_POINTER_ALLOCATE
#undef ARILES_POINTER_RESET
#undef ARILES_POINTER_CHECK_DEFINED
