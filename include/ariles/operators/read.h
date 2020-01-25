/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <vector>
#include "defaults.h"

namespace ariles
{
    namespace read
    {
        class ARILES_VISIBILITY_ATTRIBUTE Iterator
        {
            public:
                typedef int ReaderIndicatorType;
                typedef ariles::ConfigurableFlags ReadParameters;

                ReadParameters default_parameters_;

                enum SizeLimitEnforcementType
                {
                    SIZE_LIMIT_UNDEFINED = 0,
                    SIZE_LIMIT_NONE = 1,
                    SIZE_LIMIT_EQUAL = 2,
                    SIZE_LIMIT_RANGE = 3,
                    SIZE_LIMIT_MIN = 4
                };


            protected:
                /**
                 * @brief open configuration file
                 *
                 * @param[out] config_ifs
                 * @param[in] file_name
                 */
                void openFile(std::ifstream &config_ifs, const std::string& file_name)
                {
                    config_ifs.open(file_name.c_str());
                    if (!config_ifs.good())
                    {
                        std::string file_name_default = std::string(ARILES_DEFAULT_CONFIG_PREFIX) + file_name;
                        config_ifs.open(file_name_default.c_str());
                    }
                    ARILES_PERSISTENT_ASSERT(   true == config_ifs.good(),
                                                std::string("Could not open configuration file: ") + file_name.c_str());
                }


                template<int t_size_limit_type>
                std::size_t checkSize(
                        const std::size_t & /*size*/,
                        const std::size_t & /*min*/ = 0,
                        const std::size_t & /*max*/ = 0) const
                {
                    ARILES_THROW("Internal logic error.");
                }

                template<int t_size_limit_type>
                struct RelaxedSizeLimitType
                {
                    static const int value =
                        SIZE_LIMIT_EQUAL == t_size_limit_type || SIZE_LIMIT_RANGE == t_size_limit_type
                        ? SIZE_LIMIT_MIN
                        : t_size_limit_type;
                };


                virtual std::size_t getMapSize(const bool expect_empty) = 0;
                virtual std::size_t startMapImpl(const std::size_t size)
                {
                    return (size);
                }


            public:
                template<class t_Configurable>
                    void startBody(t_Configurable & /*configurable*/, ReadParameters &param)
                {
                    ARILES_TRACE_FUNCTION;
                    if (false == param.isSet(ReadParameters::PROPAGATE_ALLOW_MISSING_ENTRIES))
                    {
                        param.set(ReadParameters::DEFAULT & ReadParameters::ALLOW_MISSING_ENTRIES);
                    }
                }

                template<class t_Configurable>
                    void finishBody(t_Configurable & configurable, ReadParameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                    configurable.finalize();
                }


                template<class t_Configurable>
                    void startRoot(t_Configurable &configurable, const ReadParameters &)
                {
                    ariles::defaults::Iterator iterator;
                    configurable.arilesApply(iterator, iterator.default_parameters_);
                }

                template<class t_Configurable>
                    void finishRoot(t_Configurable &, const ReadParameters &)
                {
                }


                virtual const BridgeFlags & getBridgeFlags() const = 0;


                /**
                 * @brief Descend to the entry with the given name
                 *
                 * @param[in] child_name child node name
                 *
                 * @return true if successful.
                 */
                virtual bool descend(const std::string & child_name)
                {
                    ARILES_UNUSED_ARG(child_name)
                    return (true);
                }


                /**
                 * @brief Ascend from the current entry to its parent.
                 */
                virtual void ascend() = 0;


                template<int t_size_limit_type>
                    std::size_t startMap(
                        const std::size_t & min = 0,
                        const std::size_t & max = 0)
                {
                    return (startMapImpl( checkSize<RelaxedSizeLimitType<t_size_limit_type>::value>(
                                getMapSize(0 == max),
                                min,
                                max)));
                }

                virtual bool getMapEntryNames(std::vector<std::string> &)
                {
                    return (false);
                }

                virtual void endMap()
                {
                }


                virtual std::size_t startArray() = 0;
                virtual void shiftArray() = 0;
                virtual void endArray() = 0;



                #define ARILES_BASIC_TYPE(type) \
                        virtual void readElement(type &entry) = 0;

                ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                #undef ARILES_BASIC_TYPE
        };


        template<>
        inline std::size_t Iterator::checkSize<Iterator::SIZE_LIMIT_NONE>(
                const std::size_t & size,
                const std::size_t & /*min*/,
                const std::size_t & /*max*/) const
        {
            return (size);
        }


        template<>
        inline std::size_t Iterator::checkSize<Iterator::SIZE_LIMIT_EQUAL>(
                const std::size_t & size,
                const std::size_t & expected_size,
                const std::size_t & /*max*/) const
        {
            ARILES_ASSERT(expected_size == size, "Actual number of entries is not the same as expected.");
            return (size);
        }


        template<>
        inline std::size_t Iterator::checkSize<Iterator::SIZE_LIMIT_RANGE>(
                const std::size_t & size,
                const std::size_t & min,
                const std::size_t & max) const
        {
            ARILES_ASSERT(min <= size, "Actual number of entries is lower than expected.");
            ARILES_ASSERT(max >= size, "Actual number of entries is larger than expected.");
            return (size);
        }


        template<>
        inline std::size_t Iterator::checkSize<Iterator::SIZE_LIMIT_MIN>(
                const std::size_t & size,
                const std::size_t & min,
                const std::size_t & /*max*/) const
        {
            ARILES_ASSERT(min <= size, "Actual number of entries is lower than expected.");
            return (size);
        }


        class ARILES_VISIBILITY_ATTRIBUTE Base
        {
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
                virtual const ConfigurableFlags &getArilesConfigurableFlags() const = 0;

                virtual void arilesApply(   ariles::read::Iterator &,
                                            const ariles::read::Iterator::ReadParameters &) = 0;


                /**
                 * @todo DEPRECATED
                 * @{
                 */
                virtual void readConfigEntries( ariles::read::Iterator & iterator,
                                                const ariles::read::Iterator::ReadParameters & parameters)
                {
                    ARILES_TRACE_FUNCTION;
                    arilesApply(iterator, parameters);
                }


                /**
                 * @brief Read configuration (assuming the configuration node
                 * to be in the root).
                 *
                 * @param[in] reader configuration reader
                 */
                void readConfig(ariles::read::Iterator & reader, const ariles::ConfigurableFlags & param)
                {
                    ARILES_TRACE_FUNCTION;
                    this->readConfig(reader, this->getConfigSectionID(), param);
                }
                void readConfig(ariles::read::Iterator & reader)
                {
                    ARILES_TRACE_FUNCTION;
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
                        virtual void readConfig(ariles::read::Iterator  &   reader, \
                                                NameType                    node_name, \
                                                const ariles::read::Iterator::ReadParameters & param) = 0; \
                        void readConfig(ariles::read::Iterator  &   reader, \
                                        NameType                    node_name) \
                        { \
                            ARILES_TRACE_FUNCTION; \
                            this->readConfig(reader, node_name, this->getArilesConfigurableFlags()); \
                        }

                ARILES_READ_CONFIG(const std::string &)
                ARILES_READ_CONFIG(const char *)

                #undef ARILES_READ_CONFIG


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
                            ARILES_TRACE_FUNCTION; \
                            typename t_Bridge::Reader reader(reader_initializer); \
                            this->readConfig(reader, this->getConfigSectionID(), this->getArilesConfigurableFlags()); \
                        } \
                        template <class t_Bridge, class t_ReaderInitializer> \
                            void readConfig(InitializerType & reader_initializer, \
                                            const ariles::ConfigurableFlags & param, \
                                            typename t_Bridge::BridgeSelectorIndicatorType * = NULL) \
                        { \
                            ARILES_TRACE_FUNCTION; \
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
                            ARILES_TRACE_FUNCTION; \
                            typename t_Bridge::Reader reader(reader_initializer); \
                            this->readConfig(reader, node_name, this->getArilesConfigurableFlags()); \
                        } \
                        template <class t_Bridge, class t_ReaderInitializer> \
                            void readConfig(InitializerType     &reader_initializer, \
                                            NameType            node_name, \
                                            const ariles::ConfigurableFlags & param, \
                                            typename t_Bridge::BridgeSelectorIndicatorType * = NULL) \
                        { \
                            ARILES_TRACE_FUNCTION; \
                            typename t_Bridge::Reader reader(reader_initializer); \
                            this->readConfig(reader, node_name, param); \
                        }

                ARILES_READ_CONFIG(t_ReaderInitializer, const std::string &)
                ARILES_READ_CONFIG(const t_ReaderInitializer, const std::string &)
                ARILES_READ_CONFIG(t_ReaderInitializer, const char *)
                ARILES_READ_CONFIG(const t_ReaderInitializer, const char *)

                #undef ARILES_READ_CONFIG

                /// @}
        };
    }
}
