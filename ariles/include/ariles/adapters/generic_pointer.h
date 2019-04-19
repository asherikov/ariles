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
    template <  class t_Reader,
                typename t_Entry,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                t_Reader &reader,
                ARILES_POINTER_TYPE(t_Entry) &entry,
                const t_Flags & param)
    {
        bool is_null = true;

        ariles::ConfigurableFlags param_local = param;
        param_local.unset(ConfigurableFlags::ALLOW_MISSING_ENTRIES);

        reader.template startMap<t_Reader::SIZE_LIMIT_RANGE>(1, 2);
        readEntry(reader, is_null, "is_null", param_local);

        if (true == is_null)
        {
            ARILES_POINTER_RESET(entry);
        }
        else
        {
            ARILES_POINTER_ALLOCATE(t_Entry, entry);
            readEntry(reader, *entry, "value", param_local);
        }
        reader.endMap();
    }



    template <  class t_Writer,
                typename t_Entry,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE
        writeBody( t_Writer & writer,
                   const ARILES_POINTER_TYPE(t_Entry) &entry,
                   const t_Flags & param)
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


    template <typename t_Entry, class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE
        setDefaults(ARILES_POINTER_TYPE(t_Entry) &entry, const t_Flags & /*param*/)
    {
        ARILES_POINTER_RESET(entry);
    }
}

#undef ARILES_POINTER_TYPE
#undef ARILES_POINTER_ALLOCATE
#undef ARILES_POINTER_RESET
#undef ARILES_POINTER_CHECK_DEFINED
