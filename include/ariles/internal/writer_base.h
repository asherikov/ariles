/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "bridge_common.h"

namespace ariles
{
    class ARILES_VISIBILITY_ATTRIBUTE WriterBase
    {
        public:
            typedef int WriterIndicatorType;


        protected:
            /**
             * @brief open configuration file
             *
             * @param[out] config_ifs
             * @param[in] file_name
             */
            void openFile(std::ofstream &config_ofs, const std::string& file_name)
            {
                config_ofs.open(file_name.c_str());

                if (!config_ofs.good())
                {
                    ARILES_THROW_MSG(std::string("Could not open configuration file for writing: ") +  file_name.c_str());
                }
            }


        public:
            virtual const BridgeParameters & getBridgeParameters() const = 0;

            virtual void initRoot() {}

            virtual void flush() = 0;


            virtual void descend(const std::string &/*map_name*/) {}
            virtual void ascend() {}


            virtual void startMap(const std::size_t /*num_entries*/) {}
            virtual void endMap() {}


            virtual void startArray(const std::size_t size, const bool compact = false) = 0;
            virtual void shiftArray() {}
            virtual void endArray() {}

            virtual void startMatrix(const bool /*compact*/ = false) {}
            virtual void startMatrixRow() {}
            virtual void endMatrixRow() {}
            virtual void endMatrix() {}


            #define ARILES_BASIC_TYPE(type) \
                    virtual void writeElement(const type &entry) = 0;

            ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

            #undef ARILES_BASIC_TYPE
    };
}
