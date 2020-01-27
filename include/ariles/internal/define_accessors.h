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

    /// @todo DEPRECATED
    #ifdef ARILES_SECTION_ID
        #define ARILES_DEFAULT_ID ARILES_SECTION_ID
    #endif



    #ifndef ARILES_DOXYGEN_PROCESSING

    public:
        using ariles::Base::arilesApply;
        using ariles::Base::ariles;


        #ifdef ARILES_ENTRIES

            #define ARILES_NAMED_ENTRY(entry, name)     arilesEntryApply(visitor, entry, name, parameters);
            #define ARILES_PARENT(entry)                entry::arilesVisit(visitor, parameters);

            template<class t_Visitor, class t_Parameters>
            void arilesVisit(t_Visitor &visitor, const t_Parameters &parameters)
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
            }


            template<class t_Visitor, class t_Parameters>
            void arilesVisit(t_Visitor &visitor, const t_Parameters &parameters) const
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
            }

            #undef ARILES_PARENT
            #undef ARILES_NAMED_ENTRY


            #ifndef ARILES_AUTO_DEFAULTS
                void arilesVisit(   const ariles::defaults::Visitor & /*visitor*/,
                                    const ariles::defaults::Visitor::DefaultsParameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                    this->setDefaults();
                }
            #endif


            #define ARILES_NAMED_ENTRY(entry, name)     arilesEntryApply(visitor, entry, other.entry, name, parameters);
            #define ARILES_PARENT(entry)                entry::arilesVisit(visitor, other, parameters);

            template<class t_Visitor, class t_Parameters, class t_Other>
                void arilesVisit(const t_Visitor &visitor, t_Other & other, const t_Parameters &parameters) const
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(other);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
            }

            #undef ARILES_PARENT
            #undef ARILES_NAMED_ENTRY


            #undef ARILES_TYPED_NAMED_ENTRY
        #endif
    #endif


    public:
        // Define node name
        #ifdef ARILES_DEFAULT_ID
            const std::string & arilesDefaultID() const
            {
                static const std::string name(ARILES_DEFAULT_ID);
                return (name);
            }
        #else
            const std::string & arilesDefaultID() const
            {
                return (getConfigSectionID());
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
                    ariles::read::Visitor &reader,
                    const std::string &node_name)
            {
                ARILES_TRACE_FUNCTION;
                readConfig(reader, node_name, this->getArilesConfigurableFlags());
            }
            ARILES_CONSTRUCTOR(
                    ariles::read::Visitor &reader,
                    const std::string &node_name,
                    const ariles::ConfigurableFlags & param)
            {
                ARILES_TRACE_FUNCTION;
                readConfig(reader, node_name, param);
            }


            explicit ARILES_CONSTRUCTOR(
                    ariles::read::Visitor &reader)
            {
                ARILES_TRACE_FUNCTION;
                readConfig(reader, this->getArilesConfigurableFlags());
            }
            explicit ARILES_CONSTRUCTOR(
                    ariles::read::Visitor &reader,
                    const ariles::read::Visitor::ReadParameters & param)
            {
                ARILES_TRACE_FUNCTION;
                readConfig(reader, param);
            }
        #endif


        using ariles::CommonConfigurableBase::readConfig;

        void readConfig(ariles::read::Visitor  & reader,
                        const std::string   & node_name,
                        const ariles::ConfigurableFlags & param)
        {
            ARILES_TRACE_FUNCTION;
            ariles(reader, node_name, param);
        }

        void readConfig(ariles::read::Visitor & reader,
                        const char          * node_name,
                        const ariles::ConfigurableFlags & param)
        {
            ARILES_TRACE_FUNCTION;
            ariles(reader, node_name, param);
        }

        void arilesFinalize()
        {
            ariles::finalize::Visitor visitor;
            ariles(visitor);
        }


        using ariles::CommonConfigurableBase::writeConfig;

        void writeConfig(   ariles::write::Visitor & writer,
                            const std::string &node_name,
                            const ariles::ConfigurableFlags & param) const
        {
            ARILES_TRACE_FUNCTION;
            ariles(writer, node_name, param);
        }

        void writeConfig(   ariles::write::Visitor & writer,
                            const char *node_name,
                            const ariles::ConfigurableFlags & param) const
        {
            ARILES_TRACE_FUNCTION;
            ariles(writer, node_name, param);
        }

        std::size_t getNumberOfEntries() const
        {
            ARILES_TRACE_FUNCTION;
            ariles::count::Visitor visitor;
            arilesApply(visitor);
            return(visitor.counter_);
        }

        template<class t_Other>
            bool arilesCompare(const t_Other &other, const ariles::compare::Visitor::CompareParameters & param) const
        {
            ARILES_TRACE_FUNCTION;
            try
            {
                ariles::compare::Visitor visitor;
                ariles(visitor, other, param);
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


        ARILES_METHODS( ariles::read::Visitor,
                        const ariles::read::Visitor::ReadParameters,
                        ARILES_EMPTY_MACRO)

        ARILES_METHODS( ariles::write::Visitor,
                        const ariles::write::Visitor::WriteParameters,
                        const)

        ARILES_METHODS( const ariles::defaults::Visitor,
                        const ariles::defaults::Visitor::DefaultsParameters,
                        ARILES_EMPTY_MACRO)

        ARILES_METHODS( const ariles::finalize::Visitor,
                        const ariles::finalize::Visitor::FinalizeParameters,
                        ARILES_EMPTY_MACRO)

        ARILES_METHODS( ariles::count::Visitor,
                        const ariles::count::Visitor::CountParameters,
                        const)

        ARILES_NONVIRTUAL_METHODS(  ariles::count::Visitor,
                                    const ariles::count::Visitor::CountParameters,
                                    const)


        ARILES_METHODS_WITH_ARG(const ariles::compare::Visitor,
                                const ariles::compare::Visitor::CompareParameters,
                                const)

        #ifdef ARILES_DEFAULT_ID
            const std::string & getConfigSectionID() const
            {
                return (arilesDefaultID());
            }
        #endif


#endif //ARILES_ENABLED

#undef ARILES_DEFAULT_ID
#undef ARILES_CONSTRUCTOR
#undef ARILES_AUTO_DEFAULTS
#undef ARILES_NO_AUTO_FINALIZE
#undef ARILES_ENTRIES
#undef ARILES_CONFIGURABLE_FLAGS


#undef ARILES_SECTION_ID
