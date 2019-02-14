/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    template <class t_Reader, class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                                t_Reader  & reader,
                                ariles::CommonConfigurableBase & entry,
                                const t_Flags & param)
    {
        if (true == param.isSet(ConfigurableFlags::ALLOW_MISSING_ENTRIES))
        {
            reader.template startMap<t_Reader::SIZE_LIMIT_NONE>(entry.getNumberOfEntries());
        }
        else
        {
            reader.template startMap<t_Reader::SIZE_LIMIT_EQUAL>(entry.getNumberOfEntries());
        }
        entry.readConfigEntries(reader, param);
        reader.endMap();
    }


    /**
     * @brief Read configuration entry (an enum)
     * This function is necessary since an explicit casting to integer is needed.
     */
    template <  class t_Reader,
                typename t_Enumeration,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE readBody(
                        t_Reader & reader,
                        t_Enumeration &entry,
                        const t_Flags & /*param*/,
                        // ENABLE this function for enums
                        ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
    {
        int tmp_value = 0;
        reader.readElement(tmp_value);
        entry = static_cast<t_Enumeration> (tmp_value);
    }


    #define ARILES_BASIC_TYPE(type) \
            template <class t_Reader, class t_Flags> \
                void ARILES_VISIBILITY_ATTRIBUTE readBody( \
                                t_Reader & reader, \
                                type &entry, \
                                const t_Flags & param) \
            { \
                ARILES_UNUSED_ARG(param);\
                reader.readElement(entry);\
            }

    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

    #undef ARILES_BASIC_TYPE



    template <  class t_Reader,
                class t_Entry,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE readEntry(
                        t_Reader & reader,
                        t_Entry &entry,
                        const std::string & entry_name,
                        const t_Flags & param)
    {
        if (reader.descend(entry_name))
        {
            try
            {
                readBody(reader, entry, param);
            }
            catch(const std::exception &e)
            {
                ARILES_THROW(std::string("Failed to parse entry <")
                            + entry_name
                            + "> ||  "
                            + e.what());
            }

            reader.ascend();
        }
        else
        {
            ARILES_PERSISTENT_ASSERT(true == param.isSet(ConfigurableFlags::ALLOW_MISSING_ENTRIES),
                                 std::string("Configuration file does not contain entry '") + entry_name + "'.");
        }
    }


    // ============================================


    template <class t_Writer, class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                        t_Writer & writer,
                        const ariles::CommonConfigurableBase & entry,
                        const t_Flags & param)
    {
        writer.startMap(entry.getNumberOfEntries());
        entry.writeConfigEntries(writer, param);
        writer.endMap();
    }



    template <  class t_Writer,
                typename t_Enumeration,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                        t_Writer & writer,
                        const t_Enumeration  entry,
                        const t_Flags & /*param*/,
                        ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
    {
        int tmp_value = entry;
        writer.writeElement(tmp_value);
    }


    #define ARILES_BASIC_TYPE(type) \
            template <  class t_Writer, \
                        class t_Flags> \
                void ARILES_VISIBILITY_ATTRIBUTE writeBody( \
                                t_Writer &  writer, \
                                const type & entry, \
                                const t_Flags & param) \
            {\
                ARILES_UNUSED_ARG(param); \
                writer.writeElement(entry);\
            }

    /**
     * @brief Generate writeBody methods for basic types.
     */
    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

    #undef ARILES_BASIC_TYPE



    template <  class t_Writer,
                typename t_Entry,
                class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE writeEntry(
                        t_Writer & writer,
                        const t_Entry & entry,
                        const std::string & entry_name,
                        const t_Flags & param)
    {
        writer.descend(entry_name);
        writeBody(writer, entry, param);
        writer.ascend();
    }


    // ============================================


    template<class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(
                        ariles::CommonConfigurableBase & entry,
                        const t_Flags & /*param*/)
    {
        entry.setDefaults();
    }


    template <typename t_Enumeration, class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(
                        t_Enumeration  & entry,
                        const t_Flags & /*param*/,
                        ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
    {
        entry = static_cast<t_Enumeration>(0);
    }


    template<class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(bool & entry, const t_Flags & /*param*/)
    {
        entry = false;
    }


    template<class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(std::string & entry, const t_Flags & /*param*/)
    {
        entry = "";
    }


    #define ARILES_BASIC_TYPE(type) \
            template<class t_Flags> \
                void ARILES_VISIBILITY_ATTRIBUTE setDefaults(type & entry, const t_Flags & /*param*/) \
            { entry = 0; }

    /**
     * @brief Generate setDefaults methods for basic types.
     */
    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_INTEGER_TYPES_LIST)

    #undef ARILES_BASIC_TYPE


    template<class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(float & entry, const t_Flags & /*param*/)
    {
        entry = ARILES_DEFAULT_FLOAT_VALUE;
    }

    template<class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(double & entry, const t_Flags & /*param*/)
    {
        entry = ARILES_DEFAULT_DOUBLE_VALUE;
    }
}
