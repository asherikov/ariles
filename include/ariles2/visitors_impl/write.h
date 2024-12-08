/**
    @file
    @author  Alexander Sherikov

    @copyright 2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <fstream>


namespace ariles2
{
    namespace write
    {
        class ARILES2_VISIBILITY_PRIVATE FileVisitorImplementation
        {
        public:
            /// output file stream
            std::ofstream config_ofs_;

            /// output stream
            std::ostream *output_stream_;


        protected:
            explicit FileVisitorImplementation(const std::string &file_name)
            {
                openFile(file_name);
                output_stream_ = &config_ofs_;
            }

            explicit FileVisitorImplementation(std::ostream &output_stream)
            {
                output_stream_ = &output_stream;
            }

            /**
             * @brief open configuration file
             *
             * @param[in] file_name
             */
            void openFile(const std::string &file_name)
            {
                config_ofs_.open(file_name.c_str());

                CPPUT_PERSISTENT_ASSERT(
                        config_ofs_.good(), "Could not open configuration file for writing: ", file_name.c_str());
            }
        };
    }  // namespace write
}  // namespace ariles2
