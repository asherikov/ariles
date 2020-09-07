/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <vector>

#include "serialization.h"

#ifndef ARILES_API_VERSION
#    error "ARILES_API_VERSION is not defined, probably unhandled includion order, add explicit definition of ARILES_API_VERSION."
#endif

#if 1 == ARILES_API_VERSION
#    include "defaults.h"
#    include "postprocess.h"
#endif


namespace ariles
{
    namespace read
    {
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public serialization::Base
        {
        public:
            enum SizeLimitEnforcementType
            {
                SIZE_LIMIT_UNDEFINED = 0,
                SIZE_LIMIT_NONE = 1,
                SIZE_LIMIT_EQUAL = 2,
                SIZE_LIMIT_RANGE = 3,
                SIZE_LIMIT_MIN = 4
            };


        protected:
            template <int t_size_limit_type>
            std::size_t checkSize(
                    const std::size_t & /*size*/,
                    const std::size_t & /*min*/ = 0,
                    const std::size_t & /*max*/ = 0) const
            {
                ARILES_THROW("Internal logic error.");
            }

            template <int t_size_limit_type>
            struct RelaxedSizeLimitType
            {
                static const int value =
                        SIZE_LIMIT_EQUAL == t_size_limit_type || SIZE_LIMIT_RANGE == t_size_limit_type ?
                                SIZE_LIMIT_MIN :
                                t_size_limit_type;
            };


            virtual std::size_t getMapSize(const bool expect_empty) = 0;
            virtual std::size_t startMapImpl(const std::size_t size)
            {
                return (size);
            }

            Visitor(){};
            ~Visitor(){};


        public:
            /**
             * @brief open configuration file
             *
             * @param[out] config_ifs
             * @param[in] file_name
             */
            static void openFile(std::ifstream &config_ifs, const std::string &file_name)
            {
                config_ifs.open(file_name.c_str());
                if (!config_ifs.good())
                {
                    std::string file_name_default = file_name;
                    config_ifs.open(file_name_default.c_str());
                }
                ARILES_PERSISTENT_ASSERT(
                        true == config_ifs.good(),
                        std::string("Could not open configuration file: ") + file_name.c_str());
            }


            /**
             * @brief Descend to the entry with the given name
             *
             * @param[in] child_name child node name
             *
             * @return true if successful.
             */
            virtual bool descend(const std::string &child_name)
            {
                ARILES_TRACE_FUNCTION;
                ARILES_UNUSED_ARG(child_name)
                return (true);
            }


            /**
             * @brief Ascend from the current entry to its parent.
             */
            virtual void ascend() = 0;


            template <int t_size_limit_type>
            std::size_t startMap(const std::size_t &min = 0, const std::size_t &max = 0)
            {
                return (startMapImpl(
                        checkSize<RelaxedSizeLimitType<t_size_limit_type>::value>(getMapSize(0 == max), min, max)));
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

            virtual bool startRoot(const std::string &name)
            {
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_ENTRY(name);
                if (false == name.empty())
                {
                    return (descend(name));
                }
                return (true);
            }

            virtual void endRoot(const std::string &name)
            {
                ARILES_TRACE_FUNCTION;
                if (false == name.empty())
                {
                    ascend();
                }
            }


#define ARILES_BASIC_TYPE(type) virtual void readElement(type &entry) = 0;

            ARILES_BASIC_TYPES_LIST

#undef ARILES_BASIC_TYPE


            template <class t_Entry>
            void start(t_Entry &entry, const std::string &name, const Parameters &parameters)
            {
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_ENTRY(name);
                ARILES_TRACE_TYPE(entry);

                Parameters param = parameters;  // local modifiable copy

#if 1 == ARILES_API_VERSION
                ariles::apply<ariles::defaults::Visitor>(entry);
#endif
                if (this->startRoot(name))
                {
                    try
                    {
                        apply_read(*this, entry, param);
                    }
                    catch (const std::exception &e)
                    {
                        ARILES_THROW(std::string("Failed to parse entry <") + name + "> ||  " + e.what());
                    }

                    this->endRoot(name);
                }
                else
                {
                    ARILES_PERSISTENT_ASSERT(
                            true == param.isSet(Parameters::ALLOW_MISSING_ENTRIES),
                            std::string("Configuration file does not contain entry '") + name + "'.");
                }
            }


            template <class t_Entry>
            bool operator()(t_Entry &entry, const std::string &name, const Parameters &parameters)
            {
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_ENTRY(name);
                ARILES_TRACE_TYPE(entry);
                Parameters param = parameters;  // local modifiable copy

                if (this->descend(name))
                {
                    param.unset(Parameters::DISABLE_ALLOW_MISSING_ENTRIES);

                    try
                    {
                        apply_read(*this, entry, param);
                    }
                    catch (const std::exception &e)
                    {
                        ARILES_THROW(std::string("Failed to parse entry <") + name + "> ||  " + e.what());
                    }

                    this->ascend();
                    return (true);
                }
                else
                {
                    ARILES_PERSISTENT_ASSERT(
                            false == param.isSet(Parameters::DISABLE_ALLOW_MISSING_ENTRIES)
                                    and true == param.isSet(Parameters::ALLOW_MISSING_ENTRIES),
                            std::string("Configuration file does not contain entry '") + name + "'.");
                    return (false);
                }
            }
        };


        template <>
        inline std::size_t Visitor::checkSize<Visitor::SIZE_LIMIT_NONE>(
                const std::size_t &size,
                const std::size_t & /*min*/,
                const std::size_t & /*max*/) const
        {
            return (size);
        }


        template <>
        inline std::size_t Visitor::checkSize<Visitor::SIZE_LIMIT_EQUAL>(
                const std::size_t &size,
                const std::size_t &expected_size,
                const std::size_t & /*max*/) const
        {
            ARILES_ASSERT(expected_size == size, "Actual number of entries is not the same as expected.");
            return (size);
        }


        template <>
        inline std::size_t Visitor::checkSize<Visitor::SIZE_LIMIT_RANGE>(
                const std::size_t &size,
                const std::size_t &min,
                const std::size_t &max) const
        {
            ARILES_ASSERT(min <= size, "Actual number of entries is lower than expected.");
            ARILES_ASSERT(max >= size, "Actual number of entries is larger than expected.");
            return (size);
        }


        template <>
        inline std::size_t Visitor::checkSize<Visitor::SIZE_LIMIT_MIN>(
                const std::size_t &size,
                const std::size_t &min,
                const std::size_t & /*max*/) const
        {
            ARILES_ASSERT(min <= size, "Actual number of entries is lower than expected.");
            return (size);
        }


        class ARILES_VISIBILITY_ATTRIBUTE Base : public entry::Base<read::Visitor>
        {
        };


#ifndef ARILES_METHODS_read
#    define ARILES_METHODS_read ARILES_METHODS(read, ARILES_EMPTY_MACRO, ARILES_EMPTY_MACRO)
#endif
    }  // namespace read


    typedef read::Visitor Read;
}  // namespace ariles
