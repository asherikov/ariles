/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <vector>

namespace ariles
{
    class ARILES_VISIBILITY_ATTRIBUTE ReaderBase
    {
        public:
            typedef int ReaderIndicatorType;

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
    inline std::size_t ReaderBase::checkSize<ReaderBase::SIZE_LIMIT_NONE>(
            const std::size_t & size,
            const std::size_t & /*min*/,
            const std::size_t & /*max*/) const
    {
        return (size);
    }


    template<>
    inline std::size_t ReaderBase::checkSize<ReaderBase::SIZE_LIMIT_EQUAL>(
            const std::size_t & size,
            const std::size_t & expected_size,
            const std::size_t & /*max*/) const
    {
        ARILES_ASSERT(expected_size == size, "Actual number of entries is lower than expected.");
        return (size);
    }


    template<>
    inline std::size_t ReaderBase::checkSize<ReaderBase::SIZE_LIMIT_RANGE>(
            const std::size_t & size,
            const std::size_t & min,
            const std::size_t & max) const
    {
        ARILES_ASSERT(min <= size, "Actual number of entries is lower than expected.");
        ARILES_ASSERT(max >= size, "Actual number of entries is larger than expected.");
        return (size);
    }


    template<>
    inline std::size_t ReaderBase::checkSize<ReaderBase::SIZE_LIMIT_MIN>(
            const std::size_t & size,
            const std::size_t & min,
            const std::size_t & /*max*/) const
    {
        ARILES_ASSERT(min <= size, "Actual number of entries is lower than expected.");
        return (size);
    }
}
