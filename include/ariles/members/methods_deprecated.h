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


#ifdef ARILES_ENABLED

    #ifndef ARILES_DOXYGEN_PROCESSING

    public:
        #ifdef ARILES_ENTRIES

            #ifndef ARILES_AUTO_DEFAULTS
                void arilesVisit(   const ariles::defaults::Visitor & /*visitor*/,
                                    const ariles::defaults::Visitor::Parameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                    this->setDefaults();
                }
            #endif

            #ifndef ARILES_NO_AUTO_FINALIZE
                void arilesFinalize()
                {
                    ariles::apply<ariles::finalize::Visitor>(*this);
                }
            #endif


            std::size_t getNumberOfEntries() const
            {
                ARILES_TRACE_FUNCTION;
                ariles::count::Visitor visitor;
                ariles::apply(visitor, *this);
                return(visitor.counter_);
            }


            template<class t_Other>
                bool arilesCompare(const t_Other &other, const ariles::compare::Visitor::Parameters & param) const
            {
                ARILES_TRACE_FUNCTION;
                ariles::compare::Visitor visitor;
                ariles::apply(visitor, *this, other, arilesDefaultID(), param);
                return (visitor.equal_);
            }


        #else // ARILES_ENTRIES

            virtual void arilesVisit(   ariles::count::Visitor &visitor,
                                        const ariles::count::Visitor::Parameters &/*param*/) const
            {
                ARILES_TRACE_FUNCTION;
                visitor.counter_ += this->getNumberOfEntries();
            }

            virtual void arilesVisit(   ariles::read::Visitor &visitor,
                                        const ariles::read::Visitor::Parameters &param)
            {
                ARILES_TRACE_FUNCTION;
                readConfigEntries(visitor, param);
            }

            virtual void arilesVisit(   ariles::write::Visitor &visitor,
                                        const ariles::write::Visitor::Parameters &param) const
            {
                ARILES_TRACE_FUNCTION;
                writeConfigEntries(visitor, param);
            }

            virtual void arilesVisit(   const ariles::finalize::Visitor &/*visitor*/,
                                        const ariles::finalize::Visitor::Parameters &/*param*/)
            {
                ARILES_TRACE_FUNCTION;
                arilesFinalize();
            }

            virtual void arilesVisit(   const ariles::defaults::Visitor &/*visitor*/,
                                        const ariles::defaults::Visitor::Parameters &/*param*/)
            {
                ARILES_TRACE_FUNCTION;
                setDefaults();
            }

            template<class t_Other>
            void arilesVisit(   const ariles::compare::Visitor &/*visitor*/,
                                const t_Other & other,
                                const ariles::compare::Visitor::Parameters &param) const
            {
                ARILES_TRACE_FUNCTION;
                if (false == arilesCompare(other, param))
                {
                    ARILES_THROW("Comparison failed in " + arilesDefaultID());
                }
            }

        #endif
    #endif


    public:
        #ifdef ARILES_CONFIGURABLE_FLAGS
            virtual const ariles::ConfigurableFlags &getArilesConfigurableFlags() const
            {
                static ariles::ConfigurableFlags parameters(ARILES_CONFIGURABLE_FLAGS);
                return (parameters);
            }
        #endif
        virtual const ariles::ConfigurableFlags &arilesGetParameters(const ariles::read::Visitor &) const
        {
            return (this->getArilesConfigurableFlags());
        }
        virtual const ariles::ConfigurableFlags &arilesGetParameters(const ariles::write::Visitor &) const
        {
            return (this->getArilesConfigurableFlags());
        }


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
                    const ariles::read::Visitor::Parameters & param)
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
            ariles::apply(reader, *this, node_name, param);
        }

        void readConfig(ariles::read::Visitor & reader,
                        const char          * node_name,
                        const ariles::ConfigurableFlags & param)
        {
            ARILES_TRACE_FUNCTION;
            ariles::apply(reader, *this, node_name, param);
        }


        using ariles::CommonConfigurableBase::writeConfig;

        void writeConfig(   ariles::write::Visitor & writer,
                            const std::string &node_name,
                            const ariles::ConfigurableFlags & param) const
        {
            ARILES_TRACE_FUNCTION;
            ariles::apply(writer, *this, node_name, param);
        }

        void writeConfig(   ariles::write::Visitor & writer,
                            const char *node_name,
                            const ariles::ConfigurableFlags & param) const
        {
            ARILES_TRACE_FUNCTION;
            ariles::apply(writer, *this, node_name, param);
        }


        #ifdef ARILES_SECTION_ID
            const std::string & getConfigSectionID() const
            {
                return (arilesDefaultID());
            }
        #else
            const std::string & arilesDefaultID() const
            {
                return (getConfigSectionID());
            }
        #endif


#endif //ARILES_ENABLED

#undef ARILES_CONSTRUCTOR
#undef ARILES_AUTO_DEFAULTS
#undef ARILES_NO_AUTO_FINALIZE
#undef ARILES_CONFIGURABLE_FLAGS
#undef ARILES_SECTION_ID
