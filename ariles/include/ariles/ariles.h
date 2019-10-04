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
#include "internal/reader_base.h"
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


    // ----------------------------

    namespace ariles
    {
        /**
         * @brief Configurable base class.
         */
        class ARILES_VISIBILITY_ATTRIBUTE CommonConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CommonConfigurableBase() {}
                CommonConfigurableBase() {}


            public:
                /**
                 * @brief Return the default name of a configuration node
                 * corresponding to this class
                 *
                 * @return the name
                 *
                 * @attention Implementation of this method is added
                 * automatically upon inclusion of define_accessors.h if
                 * ARILES_SECTION_ID is defined.
                 */
                virtual const std::string & getConfigSectionID() const = 0;


                /**
                 * @brief This function is called automaticaly after reading
                 * a configuration file.
                 */
                virtual void finalize()
                {
                }

                /**
                 * @brief Automatically generated, finalize()'s Configarable
                 * members and parents.
                 */
                virtual void arilesFinalize() = 0;


                /**
                 * @brief Set members to their default values.
                 */
                virtual void setDefaults() = 0;


                /**
                 * @brief Get number of entries in the corresponding
                 * configuration node.
                 *
                 * @return number of entries
                 */
                virtual std::size_t getNumberOfEntries() const = 0;


                virtual const ConfigurableFlags &getArilesConfigurableFlags() const = 0;


                /**
                 * @brief Read configuration (assuming the configuration node
                 * to be in the root).
                 *
                 * @param[in] file_name file name
                 */
                #define ARILES_READ_CONFIG(InitializerType) \
                        template <class t_Bridge, class t_ReaderInitializer> \
                            void readConfig(InitializerType &reader_initializer, \
                                            typename t_Bridge::BridgeSelectorIndicatorType * = NULL) \
                        { \
                            typename t_Bridge::Reader reader(reader_initializer); \
                            this->readConfig(reader, this->getConfigSectionID(), this->getArilesConfigurableFlags()); \
                        } \
                        template <class t_Bridge, class t_ReaderInitializer> \
                            void readConfig(InitializerType & reader_initializer, \
                                            const ariles::ConfigurableFlags & param, \
                                            typename t_Bridge::BridgeSelectorIndicatorType * = NULL) \
                        { \
                            typename t_Bridge::Reader reader(reader_initializer); \
                            this->readConfig(reader, this->getConfigSectionID(), param); \
                        }

                ARILES_READ_CONFIG(t_ReaderInitializer)
                ARILES_READ_CONFIG(const t_ReaderInitializer)

                #undef ARILES_READ_CONFIG


                /**
                 * @brief Read configuration (assuming the configuration node
                 * to be in the root).
                 *
                 * @param[in] file_name file name
                 * @param[in] node_name node name, the default is used if empty
                 *
                 * @note Intercept implicit conversion of a pointer to bool.
                 */
                #define ARILES_READ_CONFIG(InitializerType, NameType) \
                        template <class t_Bridge, class t_ReaderInitializer> \
                            void readConfig(InitializerType     &reader_initializer, \
                                            NameType            node_name, \
                                            typename t_Bridge::BridgeSelectorIndicatorType * = NULL) \
                        { \
                            typename t_Bridge::Reader reader(reader_initializer); \
                            this->readConfig(reader, node_name, this->getArilesConfigurableFlags()); \
                        } \
                        template <class t_Bridge, class t_ReaderInitializer> \
                            void readConfig(InitializerType     &reader_initializer, \
                                            NameType            node_name, \
                                            const ariles::ConfigurableFlags & param, \
                                            typename t_Bridge::BridgeSelectorIndicatorType * = NULL) \
                        { \
                            typename t_Bridge::Reader reader(reader_initializer); \
                            this->readConfig(reader, node_name, param); \
                        }

                ARILES_READ_CONFIG(t_ReaderInitializer, const std::string &)
                ARILES_READ_CONFIG(const t_ReaderInitializer, const std::string &)
                ARILES_READ_CONFIG(t_ReaderInitializer, const char *)
                ARILES_READ_CONFIG(const t_ReaderInitializer, const char *)

                #undef ARILES_READ_CONFIG


                /**
                 * @brief Read configuration (assuming the configuration node
                 * to be in the root).
                 *
                 * @param[in] reader configuration reader
                 */
                void readConfig(ariles::ReaderBase & reader, const ariles::ConfigurableFlags & param)
                {
                    this->readConfig(reader, this->getConfigSectionID(), param);
                }
                void readConfig(ariles::ReaderBase & reader)
                {
                    this->readConfig(reader, this->getConfigSectionID(), this->getArilesConfigurableFlags());
                }


                /**
                 * @brief Read configuration (assuming the configuration node
                 * to be in the root).
                 *
                 * @param[in] reader configuration reader
                 * @param[in] node_name   node name, the default is used if empty
                 *
                 * @note Intercept implicit conversion of a pointer to bool.
                 */
                #define ARILES_READ_CONFIG(NameType) \
                        virtual void readConfig(ariles::ReaderBase  & reader, \
                                                NameType            node_name, \
                                                const ariles::ConfigurableFlags & param) = 0; \
                        void readConfig(ariles::ReaderBase  & reader, \
                                        NameType            node_name) \
                        { \
                            this->readConfig(reader, node_name, this->getArilesConfigurableFlags()); \
                        }

                ARILES_READ_CONFIG(const std::string &)
                ARILES_READ_CONFIG(const char *)

                #undef ARILES_READ_CONFIG



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

                virtual void readConfigEntries( ariles::ReaderBase & reader,
                                                const ConfigurableFlags & param) = 0;
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
