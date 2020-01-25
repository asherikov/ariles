/**
    @file
    @author  Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "internal/helpers.h"
#include "operators/defaults.h"
#include "operators/finalize.h"
#include "operators/compare.h"
#include "operators/count.h"
#include "operators/read.h"
#include "internal/writer_base.h"

// These defines are always necessary
#define ARILES_TYPED_ENTRY_(entry, type) ARILES_TYPED_NAMED_ENTRY(type, entry##_, #entry)
#define ARILES_TYPED_ENTRY(entry, type)  ARILES_TYPED_NAMED_ENTRY(type, entry, #entry)


#ifndef ARILES_DISABLE
#   define ARILES_ENABLED


    #define ARILES_ENTRY_(entry)     ARILES_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_ENTRY(entry)      ARILES_NAMED_ENTRY(entry, #entry)


    #define ARILES_WRITE_NAMED_ENTRY(entry, name)   ariles::writeEntry(writer, entry, name, param);

    #define ARILES_WRITE_ENTRY_(entry)   ARILES_WRITE_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_WRITE_ENTRY(entry)    ARILES_WRITE_NAMED_ENTRY(entry, #entry)

    #define ARILES_WRITE_PARENT(parent_class)  parent_class::writeConfigEntries(writer, param);


    #define ARILES_READ_NAMED_ENTRY(entry, name)    ariles::readEntry(reader, entry, name, param);

    #define ARILES_READ_ENTRY_(entry)    ARILES_READ_NAMED_ENTRY(entry##_, #entry);
    #define ARILES_READ_ENTRY(entry)     ARILES_READ_NAMED_ENTRY(entry, #entry);

    #define ARILES_READ_PARENT(parent_class)  parent_class::readConfigEntries(reader, param);


    #define ARILES_METHODS(Qualifier) \
        template<class t_Operator, class t_Parameters> \
            void ariles(t_Operator &op, \
                        t_Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            op.startRoot(*this, param); \
            arilesEntryApply(op, *this, this->getConfigSectionID(), param); \
            op.finishRoot(*this, param); \
        } \
        template<class t_Operator, class t_Parameters> \
            void ariles(t_Operator &op, \
                        const std::string & name, \
                        t_Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            op.startRoot(*this, param); \
            arilesEntryApply(op, *this, name, param); \
            op.finishRoot(*this, param); \
        }


    #define ARILES_APPLY_METHODS(Operator, Parameters, Qualifier) \
        virtual void arilesApply(   Operator &iterator, \
                                    Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            this->arilesIterator(iterator, param); \
        }


    #define ARILES_METHOD_WITH_ARG(Operator, Parameters) \
        template<class t_Extra> \
            void ariles(Operator &iterator, \
                        t_Extra & extra, \
                        Parameters &param) const \
        { \
            ARILES_TRACE_FUNCTION; \
            arilesIterator(iterator, extra, param); \
        } \
        template<class t_Extra> \
            void ariles(Operator &iterator, \
                        t_Extra & extra) const \
        { \
            ARILES_TRACE_FUNCTION; \
            arilesIterator(iterator, extra, iterator.default_parameters_); \
        }


    // ----------------------------

    namespace ariles
    {
        /**
         * @brief Configurable base class.
         */
        class ARILES_VISIBILITY_ATTRIBUTE CommonConfigurableBase
            :   public ariles::defaults::Base,
                public ariles::finalize::Base,
                public ariles::compare::Base,
                public ariles::count::Base,
                public ariles::read::Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CommonConfigurableBase() {}
                CommonConfigurableBase() {}


            public:
                using ariles::defaults::Base::arilesApply;
                using ariles::finalize::Base::arilesApply;
                using ariles::read::Base::arilesApply;


                template <class t_Iterator>
                    void arilesApply()
                {
                    ARILES_TRACE_FUNCTION;
                    t_Iterator iterator;
                    arilesApply(iterator);
                }
                template <class t_Iterator>
                    void arilesApply() const
                {
                    ARILES_TRACE_FUNCTION;
                    t_Iterator iterator;
                    arilesApply(iterator);
                }



                #define ARILES_WRITE_CONFIG(InitializerType) \
                        template <class t_Bridge, class t_WriterInitializer> \
                            void writeConfig(   InitializerType &writer_initializer, \
                                                typename t_Bridge::BridgeSelectorIndicatorType * = NULL) const \
                        { \
                            typename t_Bridge::Writer writer(writer_initializer); \
                            this->writeConfig(writer, this->getArilesConfigurableFlags()); \
                        } \
                        template <class t_Bridge, class t_WriterInitializer> \
                            void writeConfig(   InitializerType &writer_initializer, \
                                                const ariles::ConfigurableFlags & param, \
                                                typename t_Bridge::BridgeSelectorIndicatorType * = NULL) const \
                        { \
                            typename t_Bridge::Writer writer(writer_initializer); \
                            this->writeConfig(writer, param); \
                        }

                ARILES_WRITE_CONFIG(t_WriterInitializer)
                ARILES_WRITE_CONFIG(const t_WriterInitializer)

                #undef ARILES_WRITE_CONFIG



                /**
                 * @brief Write configuration.
                 *
                 * @param[in] file_name file name
                 * @param[in] node_name node name, the default is used if empty
                 */
                #define ARILES_WRITE_CONFIG(InitializerType, NameType) \
                        template <class t_Bridge, class t_WriterInitializer> \
                            void writeConfig(   InitializerType &writer_initializer, \
                                                NameType node_name, \
                                                typename t_Bridge::BridgeSelectorIndicatorType * = NULL) const \
                        { \
                            typename t_Bridge::Writer writer(writer_initializer); \
                            this->writeConfig(writer, node_name, this->getArilesConfigurableFlags()); \
                        } \
                        template <class t_Bridge, class t_WriterInitializer> \
                            void writeConfig(   InitializerType &writer_initializer, \
                                                NameType node_name, \
                                                const ariles::ConfigurableFlags & param, \
                                                typename t_Bridge::BridgeSelectorIndicatorType * = NULL) const \
                        { \
                            typename t_Bridge::Writer writer(writer_initializer); \
                            this->writeConfig(writer, node_name, param); \
                        }

                ARILES_WRITE_CONFIG(t_WriterInitializer, const std::string &)
                ARILES_WRITE_CONFIG(const t_WriterInitializer, const std::string &)
                ARILES_WRITE_CONFIG(t_WriterInitializer, const char *)
                ARILES_WRITE_CONFIG(const t_WriterInitializer, const char *)

                #undef ARILES_WRITE_CONFIG



                /**
                 * @brief Write configuration
                 *
                 * @param[in,out] writer configuration writer
                 */
                void writeConfig(   ariles::WriterBase & writer,
                                    const ariles::ConfigurableFlags & param) const
                {
                    this->writeConfig(writer, this->getConfigSectionID(), param);
                }
                void writeConfig(ariles::WriterBase & writer) const
                {
                    this->writeConfig(writer, this->getConfigSectionID(), this->getArilesConfigurableFlags());
                }


                /**
                 * @brief Write configuration
                 *
                 * @param[in,out] writer configuration writer
                 * @param[in] node_name   node name, the default is used if empty
                 */
                #define ARILES_WRITE_CONFIG(NameType) \
                        virtual void writeConfig(   ariles::WriterBase & writer, \
                                                    NameType node_name, \
                                                    const ariles::ConfigurableFlags & param) const = 0; \
                        void writeConfig(   ariles::WriterBase & writer, \
                                            NameType node_name) const \
                        { \
                            this->writeConfig(writer, node_name, this->getArilesConfigurableFlags()); \
                        }

                ARILES_WRITE_CONFIG(const std::string &)
                ARILES_WRITE_CONFIG(const char *)

                #undef ARILES_WRITE_CONFIG



                virtual void writeConfigEntries(ariles::WriterBase & writer,
                                                const ConfigurableFlags & param) const = 0;
        };


        class ARILES_VISIBILITY_ATTRIBUTE StrictConfigurableBase : public CommonConfigurableBase
        {
            public:
                virtual const ConfigurableFlags &getArilesConfigurableFlags() const
                {
                    static ConfigurableFlags parameters(
                            ariles::ConfigurableFlags::DEFAULT & !(ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES));
                    return (parameters);
                }
        };

        class ARILES_VISIBILITY_ATTRIBUTE RelaxedConfigurableBase : public CommonConfigurableBase
        {
            public:
                virtual const ConfigurableFlags &getArilesConfigurableFlags() const
                {
                    static ConfigurableFlags parameters(
                            ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                    return (parameters);
                }
        };

        /// Default configurable base
        class ARILES_VISIBILITY_ATTRIBUTE ConfigurableBase : public CommonConfigurableBase
        {
            public:
                virtual const ConfigurableFlags &getArilesConfigurableFlags() const
                {
                    static ConfigurableFlags parameters(ariles::ConfigurableFlags::DEFAULT);
                    return (parameters);
                }
        };
    }

#   include "adapters/basic.h"
#   include "types.h"

#else

#   define ARILES_DISABLED

    namespace ariles
    {
        // Some classes may inherit from this
        class ARILES_VISIBILITY_ATTRIBUTE StrictConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the
                 * child classes through a base pointer.
                 */
                ~StrictConfigurableBase() {}

            public:
                virtual void finalize() {};
        };


        // Some classes may inherit from this
        class ARILES_VISIBILITY_ATTRIBUTE RelaxedConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the
                 * child classes through a base pointer.
                 */
                ~RelaxedConfigurableBase() {}

            public:
                virtual void finalize() {};
        };


        // Some classes may inherit from this
        class ARILES_VISIBILITY_ATTRIBUTE ConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the
                 * child classes through a base pointer.
                 */
                ~ConfigurableBase() {}

            public:
                virtual void finalize() {};
        };
    }

#endif
