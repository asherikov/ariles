/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


class FixtureBase
{
    public:
        std::ifstream input_file_stream_;
        std::ofstream output_file_stream_;

    public:
        FixtureBase()
        {
        }

        std::ifstream & getReaderInitializer(const std::string & string_id)
        {
            if (true == input_file_stream_.is_open())
            {
                input_file_stream_.close();
            }
            input_file_stream_.open(string_id);
            if (false == input_file_stream_.good())
            {
                ARILES_THROW_MSG("Could not open file.");
            }
            return (input_file_stream_);
        }

        std::ofstream & getWriterInitializer(const std::string & string_id)
        {
            if (true == output_file_stream_.is_open())
            {
                output_file_stream_.close();
            }
            output_file_stream_.open(string_id);
            if (false == output_file_stream_.good())
            {
                ARILES_THROW_MSG("Could not open file.");
            }
            return (output_file_stream_);
        }
};
