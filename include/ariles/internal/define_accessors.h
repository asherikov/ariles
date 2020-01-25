/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
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
                ARILES_TRACE_FUNCTION;
                t_Parameters param = parameters; /// @todo something better?
                iterator.startBody(*this, param);
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                iterator.finishBody(*this, param);
            }


            template<class t_Iterator, class t_Parameters>
            void arilesIterator(t_Iterator &iterator, const t_Parameters &param) const
            {
                iterator.startBody(*this, param);
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                iterator.finishBody(*this, param);
            }

            #undef ARILES_PARENT
            #undef ARILES_NAMED_ENTRY


            #ifndef ARILES_AUTO_DEFAULTS
                void arilesIterator(const ariles::defaults::Iterator & /*iterator*/,
                                    const ariles::defaults::Iterator::DefaultsParameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                    this->setDefaults();
                }
            #endif


            #define ARILES_NAMED_ENTRY(entry, name)     arilesEntryApply(iterator, entry, other.entry, name, param);
            #define ARILES_PARENT(entry)                entry::arilesIterator(iterator, other, param);

            template<class t_Iterator, class t_Parameters, class t_Other>
                void arilesIterator(const t_Iterator &iterator, t_Other & other, const t_Parameters &parameters) const
            {
                t_Parameters param = parameters; /// @todo something better?
                iterator.startBody(*this, other, param);
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
                iterator.finishBody(*this, other, param);
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


            template<class t_Other>
                bool arilesCompare(const t_Other &other, const ariles::compare::Iterator::CompareParameters & param) const
            {
                ARILES_TRACE_FUNCTION;
                try
                {
                    ariles::compare::Iterator iterator;
                    ariles(iterator, other, param);
                    return (true);
                }
                catch (std::exception &e)
                {
                    if (true == param.throw_on_error_)
                    {
                        ARILES_THROW(std::string("Comparison failed: ") + e.what());
                    }
                    return (false);
                }
            }
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
                    ariles::read::Iterator &reader,
                    const std::string &node_name)
            {
                ARILES_TRACE_FUNCTION;
                readConfig(reader, node_name, this->getArilesConfigurableFlags());
            }
            ARILES_CONSTRUCTOR(
                    ariles::read::Iterator &reader,
                    const std::string &node_name,
                    const ariles::ConfigurableFlags & param)
            {
                ARILES_TRACE_FUNCTION;
                readConfig(reader, node_name, param);
            }


            explicit ARILES_CONSTRUCTOR(
                    ariles::read::Iterator &reader)
            {
                ARILES_TRACE_FUNCTION;
                readConfig(reader, this->getArilesConfigurableFlags());
            }
            explicit ARILES_CONSTRUCTOR(
                    ariles::read::Iterator &reader,
                    const ariles::read::Iterator::ReadParameters & param)
            {
                ARILES_TRACE_FUNCTION;
                readConfig(reader, param);
            }
        #endif

        ARILES_METHODS(const)
        ARILES_METHODS(ARILES_EMPTY_MACRO)

        using ariles::CommonConfigurableBase::readConfig;

        void readConfig(ariles::read::Iterator  & reader,
                        const std::string   & node_name,
                        const ariles::ConfigurableFlags & param)
        {
            ARILES_TRACE_FUNCTION;
            ariles(reader, node_name, param);
        }

        void readConfig(ariles::read::Iterator & reader,
                        const char          * node_name,
                        const ariles::ConfigurableFlags & param)
        {
            ARILES_TRACE_FUNCTION;
            ariles(reader, node_name, param);
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

        std::size_t getNumberOfEntries() const
        {
            ARILES_TRACE_FUNCTION;
            ariles::count::Iterator iterator;
            arilesApply(iterator, iterator.default_parameters_);
            return(iterator.counter_);
        }


        ARILES_APPLY_METHODS(   ariles::read::Iterator,
                                const ariles::read::Iterator::ReadParameters,
                                ARILES_EMPTY_MACRO)

        ARILES_APPLY_METHODS(   const ariles::defaults::Iterator,
                                const ariles::defaults::Iterator::DefaultsParameters,
                                ARILES_EMPTY_MACRO)

        ARILES_APPLY_METHODS(   const ariles::finalize::Iterator,
                                const ariles::finalize::Iterator::FinalizeParameters,
                                ARILES_EMPTY_MACRO)

        ARILES_APPLY_METHODS(   ariles::count::Iterator,
                                const ariles::count::Iterator::CountParameters,
                                const)

        /**
         * @{
         * Some operators, e.g., entry counter, do not have virtual
         * arilesApply() in the base, so these templates must be defined here
         * and not in the ConfigurableBase class.
         */
        template <class t_Iterator>
            void arilesApply(t_Iterator &iterator)
        {
            ARILES_TRACE_FUNCTION;
            arilesApply(iterator, iterator.default_parameters_);
        }
        template <class t_Iterator>
            void arilesApply(t_Iterator &iterator) const
        {
            ARILES_TRACE_FUNCTION;
            arilesApply(iterator, iterator.default_parameters_);
        }
        /// @}


        ARILES_METHOD_WITH_ARG( const ariles::compare::Iterator,
                                const ariles::compare::Iterator::CompareParameters)

#endif //ARILES_ENABLED

#undef ARILES_SECTION_ID
#undef ARILES_CONSTRUCTOR
#undef ARILES_AUTO_DEFAULTS
#undef ARILES_NO_AUTO_FINALIZE
#undef ARILES_ENTRIES
#undef ARILES_CONFIGURABLE_FLAGS
