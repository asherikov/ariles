/**
    @file
    @author  Alexander Sherikov

    @copyright 2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <vector>
#include <fstream>


namespace ariles2
{
    namespace read
    {
        class ARILES2_VISIBILITY_PRIVATE FileVisitorImplementation
        {
        protected:
            std::vector<std::ifstream> config_ifs_;
            std::vector<std::istream *> input_streams_;

        protected:
            FileVisitorImplementation() = default;

            explicit FileVisitorImplementation(const std::vector<std::string> &file_names)
            {
                for (const std::string &file_name : file_names)
                {
                    config_ifs_.emplace_back();
                    config_ifs_.back().open(file_name.c_str());
                    if (!config_ifs_.back().good())
                    {
                        const std::string file_name_default = file_name;
                        config_ifs_.back().open(file_name_default.c_str());
                    }
                    CPPUT_PERSISTENT_ASSERT(
                            config_ifs_.back().good(), "Could not open configuration file: ", file_name.c_str());

                    input_streams_.emplace_back(&config_ifs_.back());
                }
            }

            explicit FileVisitorImplementation(const std::vector<std::istream *> &input_streams)
            {
                input_streams_ = input_streams;
            }


            explicit FileVisitorImplementation(const std::string &file_name)
              : FileVisitorImplementation(std::vector{ file_name })
            {
            }

            explicit FileVisitorImplementation(std::istream &input_stream)
              : FileVisitorImplementation(std::vector{ &input_stream })
            {
            }
        };
    }  // namespace read
}  // namespace ariles2
