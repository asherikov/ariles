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


namespace ariles2
{
    namespace read
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public serialization::Parameters
        {
        public:
            enum MissingEntries
            {
                MISSING_ENTRIES_DISABLE = 0,
                MISSING_ENTRIES_ENABLE = 1,
                MISSING_ENTRIES_ENABLE_OVERRIDE = 2
            } missing_entries_;


        public:
            Parameters()
            {
                missing_entries_ = MISSING_ENTRIES_DISABLE;
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public serialization::Base<Parameters>
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


        public:
            bool override_missing_entries_locally_;


        protected:
            template <int t_size_limit_type>
            std::size_t checkSize(
                    const std::size_t & /*size*/,
                    const std::size_t & /*min*/ = 0,
                    const std::size_t & /*max*/ = 0) const
            {
                ARILES2_THROW("Internal logic error.");
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
                ARILES2_PERSISTENT_ASSERT(
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
                ARILES2_TRACE_FUNCTION;
                ARILES2_UNUSED_ARG(child_name)
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
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_ENTRY(name);

                override_missing_entries_locally_ = false;
                if (false == name.empty())
                {
                    return (descend(name));
                }
                return (true);
            }

            virtual void endRoot(const std::string &name)
            {
                ARILES2_TRACE_FUNCTION;
                if (false == name.empty())
                {
                    ascend();
                }
            }


#define ARILES2_BASIC_TYPE(type) virtual void readElement(type &entry) = 0;

            ARILES2_BASIC_TYPES_LIST

#undef ARILES2_BASIC_TYPE


            template <class t_Entry>
            void start(t_Entry &entry, const std::string &name, const Parameters &parameters)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_ENTRY(name);
                ARILES2_TRACE_TYPE(entry);

                Parameters param = parameters;  // local modifiable copy

                if (this->startRoot(name))
                {
                    try
                    {
                        apply_read(*this, entry, param);
                    }
                    catch (const std::exception &e)
                    {
                        ARILES2_THROW(std::string("Failed to parse entry <") + name + "> ||  " + e.what());
                    }

                    this->endRoot(name);
                }
                else
                {
                    ARILES2_PERSISTENT_ASSERT(
                            Parameters::MISSING_ENTRIES_DISABLE != param.missing_entries_,
                            std::string("Configuration file does not contain entry '") + name + "'.");
                }
            }


            template <class t_Entry>
            bool operator()(t_Entry &entry, const std::string &name, const Parameters &parameters)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_ENTRY(name);
                ARILES2_TRACE_TYPE(entry);
                Parameters param = parameters;  // local modifiable copy

                if (this->descend(name))
                {
                    override_missing_entries_locally_ = false;

                    try
                    {
                        apply_read(*this, entry, param);
                    }
                    catch (const std::exception &e)
                    {
                        ARILES2_THROW(std::string("Failed to parse entry <") + name + "> ||  " + e.what());
                    }

                    this->ascend();
                    return (true);
                }
                else
                {
                    ARILES2_PERSISTENT_ASSERT(
                            false == override_missing_entries_locally_
                                    and Parameters::MISSING_ENTRIES_DISABLE != param.missing_entries_,
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
            ARILES2_ASSERT(expected_size == size, "Actual number of entries is not the same as expected.");
            return (size);
        }


        template <>
        inline std::size_t Visitor::checkSize<Visitor::SIZE_LIMIT_RANGE>(
                const std::size_t &size,
                const std::size_t &min,
                const std::size_t &max) const
        {
            ARILES2_ASSERT(min <= size, "Actual number of entries is lower than expected.");
            ARILES2_ASSERT(max >= size, "Actual number of entries is larger than expected.");
            return (size);
        }


        template <>
        inline std::size_t Visitor::checkSize<Visitor::SIZE_LIMIT_MIN>(
                const std::size_t &size,
                const std::size_t &min,
                const std::size_t & /*max*/) const
        {
            ARILES2_ASSERT(min <= size, "Actual number of entries is lower than expected.");
            return (size);
        }


        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::Base<read::Visitor>
        {
        };


#define ARILES2_VISIT_read
#define ARILES2_METHODS_read ARILES2_METHODS(read, ARILES2_EMPTY_MACRO, ARILES2_EMPTY_MACRO)
#define ARILES2_BASE_METHODS_read ARILES2_BASE_METHODS(read)
    }  // namespace read


    typedef read::Visitor Read;
}  // namespace ariles2
