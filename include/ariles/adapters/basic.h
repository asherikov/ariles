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
    namespace adapter
    {
        template <class t_Reader>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                                    t_Reader  & reader,
                                    ariles::CommonConfigurableBase & entry,
                                    const ariles::ConfigurableFlags & param)
        {
            entry.setDefaults();
            reader.template startMap<t_Reader::SIZE_LIMIT_EQUAL>(entry.getNumberOfEntries());
            entry.readConfigEntries(reader, param);
            reader.endMap();
        }


        /**
         * @brief Read configuration entry (an enum)
         * This function is necessary since an explicit casting to integer is needed.
         */
        template <  class t_Reader,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                            t_Reader & reader,
                            t_Enumeration &entry,
                            const ariles::ConfigurableFlags & /*param*/,
                            // ENABLE this function for enums
                            ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            int tmp_value = 0;
            reader.readElement(tmp_value);
            entry = static_cast<t_Enumeration> (tmp_value);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <  class t_Reader> \
                    void ARILES_VISIBILITY_ATTRIBUTE readBody( \
                                    t_Reader & reader, \
                                    type &entry, \
                                    const ariles::ConfigurableFlags & param) \
                { \
                    ARILES_IGNORE_UNUSED(param);\
                    reader.readElement(entry);\
                }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE



        template <  class t_Reader,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE readEntry(
                            t_Reader & reader,
                            t_Entry &entry,
                            const std::string & entry_name,
                            const ariles::ConfigurableFlags & param)
        {
            if (reader.descend(entry_name))
            {
                try
                {
                    readBody(reader, entry, param);
                }
                catch(const std::exception &e)
                {
                    ARILES_THROW_MSG(   std::string("Failed to parse entry <")
                                        + entry_name
                                        + "> ||  "
                                        + e.what());
                }

                reader.ascend();
            }
            else
            {
                if (false == param.isSet(ConfigurableFlags::ALLOW_MISSING_ENTRIES))
                {
                    ARILES_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                }
            }
        }


        // ============================================


        template <class t_Writer>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                            t_Writer & writer,
                            const ariles::CommonConfigurableBase & entry,
                            const ariles::ConfigurableFlags & param)
        {
            writer.startMap(entry.getNumberOfEntries());
            entry.writeConfigEntries(writer, param);
            writer.endMap();
        }



        template <  class t_Writer,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                            t_Writer & writer,
                            const t_Enumeration  entry,
                            const ariles::ConfigurableFlags & /*param*/,
                            ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            int tmp_value = entry;
            writer.writeElement(tmp_value);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <class t_Writer> \
                    void ARILES_VISIBILITY_ATTRIBUTE writeBody( \
                                    t_Writer &  writer, \
                                    const type & entry, \
                                    const ariles::ConfigurableFlags & param) \
                {\
                    ARILES_IGNORE_UNUSED(param); \
                    writer.writeElement(entry);\
                }

        /**
         * @brief Generate writeBody methods for basic types.
         */
        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE



        template <  class t_Writer,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE writeEntry(
                            t_Writer & writer,
                            const t_Entry & entry,
                            const std::string & entry_name,
                            const ariles::ConfigurableFlags & param)
        {
            writer.descend(entry_name);
            writeBody(writer, entry, param);
            writer.ascend();
        }


        // ============================================


        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(
                        ariles::CommonConfigurableBase & entry)
        {
            entry.setDefaults();
        }


        template <typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE setDefaults(
                            t_Enumeration  & entry,
                            ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            entry = static_cast<t_Enumeration>(0);
        }


        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(bool & entry)
        {
            entry = false;
        }


        void ARILES_VISIBILITY_ATTRIBUTE setDefaults(std::string & entry)
        {
            entry = "";
        }


        #define ARILES_BASIC_TYPE(type) \
                    void ARILES_VISIBILITY_ATTRIBUTE setDefaults(type & entry) \
                { entry = 0; }

        /**
         * @brief Generate setDefaults methods for basic types.
         */
        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_INTEGER_TYPES_LIST)

        #undef ARILES_BASIC_TYPE


        #define ARILES_BASIC_TYPE(type) \
                    void ARILES_VISIBILITY_ATTRIBUTE setDefaults(type & entry) \
                { entry = 0.0; }

        /**
         * @brief Generate setDefaults methods for basic types.
         */
        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_REAL_TYPES_LIST)

        #undef ARILES_BASIC_TYPE
    }
}
