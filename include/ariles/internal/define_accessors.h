/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @copyright 2017-2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Inclusion of this file results in generation of functions which
    read and write entries 'ARILES_ENTRIES' defined in the including
    header from / to a configuration file.
*/

#ifndef ARILES_DOXYGEN_PROCESSING
    public:
        #ifdef ARILES_ENTRIES
            #define ARILES_NAMED_ENTRY(entry, name)
            #define ARILES_PARENT(entry)
            #define ARILES_TYPED_NAMED_ENTRY(type, entry, name)  type    entry;

            ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)

            #undef ARILES_NAMED_ENTRY
            #undef ARILES_PARENT
            #undef ARILES_TYPED_NAMED_ENTRY

            #define ARILES_TYPED_NAMED_ENTRY(type, entry, name)  ARILES_NAMED_ENTRY(entry, name)
        #endif
#endif


#ifdef ARILES_ENABLED


    #ifndef ARILES_DOXYGEN_PROCESSING

    public:
        #ifdef ARILES_ENTRIES

        // Define write method

            #define ARILES_NAMED_ENTRY(entry, name)     ARILES_WRITE_NAMED_ENTRY(entry, name)
            #define ARILES_PARENT(entry)                ARILES_WRITE_PARENT(entry);

            void writeConfigEntries(ariles::WriterBase & writer,
                                    const ariles::ConfigurableFlags & param) const
            {
                ARILES_UNUSED_ARG(writer);
                ARILES_UNUSED_ARG(param);
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
            }

            #undef ARILES_NAMED_ENTRY
            #undef ARILES_PARENT

        // Define read method

            #define ARILES_NAMED_ENTRY(entry, name)     ARILES_READ_NAMED_ENTRY(entry, name)
            #define ARILES_PARENT(entry)                ARILES_READ_PARENT(entry);

            void readConfigEntries( ariles::ReaderBase & reader,
                                    const ariles::ConfigurableFlags & parameters)
            {
                ariles::ConfigurableFlags param = parameters;
                if (false == param.isSet(ariles::ConfigurableFlags::PROPAGATE_ALLOW_MISSING_ENTRIES))
                {
                    param.set(ariles::ConfigurableFlags::DEFAULT & ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                }
                ARILES_UNUSED_ARG(reader);
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                finalize();
            }

            #undef ARILES_NAMED_ENTRY
            #undef ARILES_PARENT

        // Define initialization method

            #ifdef ARILES_AUTO_DEFAULTS
                #define ARILES_NAMED_ENTRY(entry, name)  ariles::setDefaults(entry, this->getArilesConfigurableFlags());
                #define ARILES_PARENT(entry)             entry::setDefaults();

                void setDefaults()
                {
                    ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                }

                #undef ARILES_NAMED_ENTRY
                #undef ARILES_PARENT
            #endif


            #ifndef ARILES_NO_AUTO_FINALIZE
                #define ARILES_NAMED_ENTRY(entry, name)  ARILES_TRACE_ENTRY(entry); ariles::finalize(entry, ariles::ArilesNamespaceLookupTrigger());
                #define ARILES_PARENT(entry)             ARILES_TRACE_ENTRY(entry); entry::arilesFinalize();

                void arilesFinalize()
                {
                    ARILES_TRACE_FUNCTION;
                    ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                    this->finalize();
                }

                #undef ARILES_NAMED_ENTRY
                #undef ARILES_PARENT
            #endif


    protected:
        // Count number of entries and define a function, which returns it.

            #define ARILES_NAMED_ENTRY(entry, name)  +1
            #define ARILES_PARENT(entry)       +entry::getNumberOfEntries()

            std::size_t getNumberOfEntries() const
            {
                static const std::size_t    num_entries = (0 ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES) );
                return(num_entries);
            }

            #undef ARILES_NAMED_ENTRY
            #undef ARILES_PARENT


            #undef ARILES_TYPED_NAMED_ENTRY
        #endif
    #endif


    public:
        // Define node name
        #ifdef ARILES_SECTION_ID
            const std::string & getConfigSectionID() const
            {
                static const std::string name(ARILES_SECTION_ID);
                return (name);
            }
        #endif


        #ifdef ARILES_CONFIGURABLE_FLAGS
            virtual const ariles::ConfigurableFlags &getArilesConfigurableFlags() const
            {
                static ariles::ConfigurableFlags parameters(ARILES_CONFIGURABLE_FLAGS);
                return (parameters);
            }
        #endif


        // generate methods which accept ConfigurableFlags
        #define ARILES_CONFIGURABLE_PARAMETERS_ARG              , const ariles::ConfigurableFlags & param
        #define ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA   ARILES_CONFIGURABLE_PARAMETERS_ARG,
        #define ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE        param
        #include "define_accessors_readwrite.h"
        #undef ARILES_CONFIGURABLE_PARAMETERS_ARG
        #undef ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
        #undef ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE

        // generate methods which use default ConfigurableFlags
        #define ARILES_CONFIGURABLE_PARAMETERS_ARG
        #define ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA   ,
        #define ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE        this->getArilesConfigurableFlags()
        #include "define_accessors_readwrite.h"
        #undef ARILES_CONFIGURABLE_PARAMETERS_ARG
        #undef ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
        #undef ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE


#endif //ARILES_ENABLED

#undef ARILES_SECTION_ID
#undef ARILES_CONSTRUCTOR
#undef ARILES_AUTO_DEFAULTS
#undef ARILES_NO_AUTO_FINALIZE
#undef ARILES_ENTRIES
#undef ARILES_CONFIGURABLE_FLAGS
