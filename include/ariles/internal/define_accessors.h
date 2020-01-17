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
        using ariles::CommonConfigurableBase::arilesApply;


        #ifdef ARILES_ENTRIES

            #define ARILES_NAMED_ENTRY(entry, name)     arilesEntryApply(iterator, entry, name, param);
            #define ARILES_PARENT(entry)                entry::arilesIterator(iterator, param);

            template<class t_Iterator, class t_Parameters>
            void arilesIterator(t_Iterator &iterator, const t_Parameters &parameters)
            {
                t_Parameters param = parameters; /// @todo something better?
                iterator.start(*this, param);
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                iterator.finish(*this, param);
            }


            template<class t_Iterator, class t_Parameters>
            void arilesIterator(t_Iterator &iterator, const t_Parameters &param) const
            {
                iterator.start(*this, param);
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                iterator.finish(*this, param);
            }

            #undef ARILES_PARENT
            #undef ARILES_NAMED_ENTRY



        // Define write method

            #define ARILES_NAMED_ENTRY(entry, name)     ARILES_WRITE_NAMED_ENTRY(entry, name)
            #define ARILES_PARENT(entry)                ARILES_WRITE_PARENT(entry);

            void writeConfigEntries(ariles::WriterBase & writer,
                                    const ariles::WriterBase::Parameters & param) const
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
                                    const ariles::ReaderBase::Parameters & parameters)
            {
                ariles::ReaderBase::Parameters param = parameters; /// @todo something better?
                reader.start(*this, param);
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                reader.finish(*this, param);
            }

            #undef ARILES_NAMED_ENTRY
            #undef ARILES_PARENT



        // Define initialization method

            #ifdef ARILES_AUTO_DEFAULTS
                void arilesApply(   ariles::defaults::Iterator &iterator,
                                    const ariles::defaults::Iterator::DefaultsParameters &param)
                {
                    ARILES_TRACE_FUNCTION;
                    arilesIterator(iterator, param);
                }
            #else
                void arilesApply(   ariles::defaults::Iterator & /*iterator*/,
                                    const ariles::defaults::Iterator::DefaultsParameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                    this->setDefaults();
                }
            #endif


            #ifndef ARILES_NO_AUTO_FINALIZE
                void arilesApply(   ariles::finalize::Iterator &iterator,
                                    const ariles::finalize::Iterator::FinalizeParameters &param)
                {
                    arilesIterator(iterator, param);
                }
            #endif


        // define comparison method.

            #define ARILES_NAMED_ENTRY(entry, name) \
                    ARILES_TRACE_ENTRY(entry); \
                    try \
                    { \
                        result = ariles::compare(entry, other.entry, param); \
                    } \
                    catch (const std::exception & e) \
                    { \
                        ARILES_THROW("Comparison failed for entry: " #entry " // " + std::string(e.what())); \
                    } \
                    if (false == result) \
                    { \
                        if (true == param.throw_on_error_)\
                        { \
                            ARILES_THROW("Comparison failed for entry: " #entry); \
                        } \
                        return (false); \
                    };

            #define ARILES_PARENT(entry) \
                    ARILES_TRACE_ENTRY(entry); \
                    try \
                    { \
                        result = entry::arilesCompare(other, param); \
                    } \
                    catch (const std::exception & e) \
                    { \
                        ARILES_THROW("Comparison failed for entry: " #entry " // " + std::string(e.what())); \
                    } \
                    if (false == result) \
                    { \
                        if (true == param.throw_on_error_)\
                        { \
                            ARILES_THROW("Comparison failed for entry: " #entry); \
                        } \
                        return (false); \
                    }

            template<class t_Other>
            bool arilesCompare(const t_Other &other, const ariles::ComparisonParameters &param) const
            {
                ARILES_TRACE_FUNCTION;
                bool result = true;
                if (true == param.compare_number_of_entries_)
                {
                    if (getNumberOfEntries() != other.getNumberOfEntries())
                    {
                        if (true == param.throw_on_error_)
                        {
                            ARILES_THROW("Comparison failed: dfferent number of entries.");
                        }
                        return (false);
                    }
                }
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)

                return (result);
            }

            #undef ARILES_NAMED_ENTRY
            #undef ARILES_PARENT


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

        #ifdef ARILES_CONSTRUCTOR
            /**
             * Define constructors for the given class.
             */
            ARILES_CONSTRUCTOR(
                    ariles::ReaderBase &reader,
                    const std::string &node_name)
            {
                readConfig(reader, node_name, this->getArilesConfigurableFlags());
            }
            ARILES_CONSTRUCTOR(
                    ariles::ReaderBase &reader,
                    const std::string &node_name,
                    const ariles::ConfigurableFlags & param)
            {
                readConfig(reader, node_name, param);
            }


            explicit ARILES_CONSTRUCTOR(
                    ariles::ReaderBase &reader)
            {
                readConfig(reader, this->getArilesConfigurableFlags());
            }
            explicit ARILES_CONSTRUCTOR(
                    ariles::ReaderBase &reader,
                    const ariles::ConfigurableFlags & param)
            {
                readConfig(reader, param);
            }
        #endif


        using ariles::CommonConfigurableBase::readConfig;

        void readConfig(ariles::ReaderBase  & reader,
                        const std::string   & node_name,
                        const ariles::ConfigurableFlags & param)
        {
            ariles::CommonConfigurableBase::arilesApply<ariles::defaults::Iterator>();
            ariles::readEntry(reader, *this, node_name, param);
        }

        void readConfig(ariles::ReaderBase  & reader,
                        const char          * node_name,
                        const ariles::ConfigurableFlags & param)
        {
            ariles::CommonConfigurableBase::arilesApply<ariles::defaults::Iterator>();
            ariles::readEntry(reader, *this, node_name, param);
        }


        using ariles::CommonConfigurableBase::writeConfig;

        void writeConfig(   ariles::WriterBase & writer,
                            const std::string &node_name,
                            const ariles::ConfigurableFlags & param) const
        {
            writer.initRoot();
            ariles::writeEntry(writer, *this, node_name, param);
            writer.flush();
        }

        void writeConfig(   ariles::WriterBase & writer,
                            const char *node_name,
                            const ariles::ConfigurableFlags & param) const
        {
            writer.initRoot();
            ariles::writeEntry(writer, *this, node_name, param);
            writer.flush();
        }


#endif //ARILES_ENABLED

#undef ARILES_SECTION_ID
#undef ARILES_CONSTRUCTOR
#undef ARILES_AUTO_DEFAULTS
#undef ARILES_NO_AUTO_FINALIZE
#undef ARILES_ENTRIES
#undef ARILES_CONFIGURABLE_FLAGS
