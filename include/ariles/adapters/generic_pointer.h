/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

namespace ariles
{
    template <  class t_Reader,
                typename t_Entry,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                t_Reader &reader,
                ARILES_POINTER_TYPE<t_Entry> &entry,
                const t_Flags & param)
    {
        bool is_null = true;

        ariles::ConfigurableFlags param_local = param;
        param_local.unset(ConfigurableFlags::ALLOW_MISSING_ENTRIES);

        reader.template startMap<t_Reader::SIZE_LIMIT_RANGE>(1, 2);
        readEntry(reader, is_null, "is_null", param_local);

        if (true == is_null)
        {
            ARILES_POINTER_HANDLER<t_Entry>::reset(entry);
        }
        else
        {
            ARILES_POINTER_HANDLER<t_Entry>::allocate(entry);
            readEntry(reader, *entry, "value", param_local);
        }
        reader.endMap();
    }



    template <  class t_Writer,
                typename t_Entry,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE
        writeBody( t_Writer & writer,
                   const ARILES_POINTER_TYPE<t_Entry> &entry,
                   const t_Flags & param)
    {
        bool is_null = true;

        if (ARILES_POINTER_HANDLER<t_Entry>::isNull(entry))
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
        setDefaults(    ARILES_POINTER_TYPE<t_Entry> &entry,
                        const t_Flags & /*param*/)
    {
        ARILES_TRACE_FUNCTION;
        ARILES_POINTER_HANDLER<t_Entry>::reset(entry);
    }


    template <typename t_Entry>
        void ARILES_VISIBILITY_ATTRIBUTE
        finalize(   ARILES_POINTER_TYPE<t_Entry> &entry,
                    const ArilesNamespaceLookupTrigger &trigger)
    {
        ARILES_TRACE_FUNCTION;
        if (false == (ARILES_POINTER_HANDLER<t_Entry>::isNull(entry)))
        {
            finalize(*entry, trigger);
        }
    }
}

#undef ARILES_POINTER_HANDLER
#undef ARILES_POINTER_TYPE
