/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


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
             * @param[out] config_ofs
             * @param[in] file_name
             */
            void openFile(std::ofstream &config_ofs, const std::string& file_name)
            {
                config_ofs.open(file_name.c_str());

                ARILES_PERSISTENT_ASSERT(   true == config_ofs.good(), 
                                        std::string("Could not open configuration file for writing: ") + file_name.c_str());
            }


        public:
            virtual const BridgeFlags & getBridgeFlags() const = 0;

			/**
			 * @brief Starts a nested map in the configuration file
			 */
            virtual void initRoot() {}

            /**
             * @brief Flush the configuration to the output
             */
            virtual void flush() = 0;


            /**
             * @brief Starts a nested map in the configuration file
             *
             * @param[in] map_name name of the map
             */
            virtual void descend(const std::string &map_name) {ARILES_UNUSED_ARG(map_name)}
            virtual void ascend() {}


            /**
             * @brief Starts a nested map in the configuration file
             *
             * @param[in] num_entries number of child entries
             */
            virtual void startMap(const std::size_t num_entries) {ARILES_UNUSED_ARG(num_entries)}

            /**
             * @brief Ends a nested map in the configuration file
             */
            virtual void endMap() {}


            virtual void startArray(const std::size_t size, const bool compact = false) = 0;
            virtual void shiftArray() {}
            virtual void endArray() {}

            virtual void startMatrix(const bool compact = false) {ARILES_UNUSED_ARG(compact)}
            virtual void startMatrixRow() {}
            virtual void endMatrixRow() {}
            virtual void endMatrix() {}


            #define ARILES_BASIC_TYPE(type) \
                    virtual void writeElement(const type &entry) = 0;

            ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

            #undef ARILES_BASIC_TYPE
    };
}
