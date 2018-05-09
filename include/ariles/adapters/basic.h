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
    namespace reader
    {
        /**
         * @brief Read configuration entry
         *
         * @tparam t_Entry type of the entry
         *
         * @param[out] entry     configuration parameter
         */
        template <class t_Reader>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                                    t_Reader  & reader,
                                    ariles::CommonConfigurableBase & entry,
                                    const ariles::ConfigurableParameters & param)
        {
            entry.setDefaults();
            entry.readConfigEntries(reader, param);
        }


        /**
         * @brief Read configuration entry (an enum)
         * This function is necessary since an explicit casting to integer is needed.
         *
         * @tparam t_Entry type of the entry
         *
         * @param[out] entry     configuration parameter
         */
        template <  class t_Reader,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                            t_Reader & reader,
                            t_Enumeration &entry,
                            const ariles::ConfigurableParameters & /*param*/,
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
                                    const ariles::ConfigurableParameters & param) \
                { \
                    ARILES_IGNORE_UNUSED(param);\
                    reader.readElement(entry);\
                }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE



        /**
         * @brief Read configuration entry
         *
         * @tparam t_Entry type of the entry
         *
         * @param[out] entry      configuration parameter
         * @param[in]  entry_name name of the configuration parameter
         */
        template <  class t_Reader,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE readEntry(
                            t_Reader & reader,
                            t_Entry &entry,
                            const std::string & entry_name,
                            const ariles::ConfigurableParameters & param)
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
                if (param.crash_on_missing_entry_)
                {
                    ARILES_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                }
            }
        }
    }


    namespace writer
    {
        /**
         * @brief Write a configuration entry (enum)
         *
         * @tparam t_EnumType type of the enum
         *
         * @param[in] entry      data
         * @param[in] entry_name name
         */
        template <  class t_Writer,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                            t_Writer & writer,
                            const t_Enumeration  entry,
                            const ariles::ConfigurableParameters & /*param*/,
                            ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            int tmp_value = entry;
            writer.writeElement(tmp_value);
        }


        template <class t_Writer>
            void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                            t_Writer & writer,
                            const ariles::CommonConfigurableBase & entry,
                            const ariles::ConfigurableParameters & param)
        {
            writer.startMap(entry.getNumberOfEntries());
            entry.writeConfigEntries(writer, param);
            writer.endMap();
        }



        #define ARILES_BASIC_TYPE(type) \
                template <class t_Writer> \
                    void ARILES_VISIBILITY_ATTRIBUTE writeBody( \
                                    t_Writer &  writer, \
                                    const type & entry, \
                                    const ariles::ConfigurableParameters & param) \
                {\
                    ARILES_IGNORE_UNUSED(param); \
                    writer.writeElement(entry);\
                }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE



        template <  class t_Writer,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE writeEntry(
                            t_Writer & writer,
                            const t_Entry & entry,
                            const std::string & entry_name,
                            const ariles::ConfigurableParameters & param)
        {
            writer.descend(entry_name);
            writeBody(writer, entry, param);
            writer.ascend();
        }
    }
}
