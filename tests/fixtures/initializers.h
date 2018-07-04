/**
    @file
    @author  Alexander Sherikov
    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace initializers
{
    class FilenameInitializer
    {
        public:
            FilenameInitializer()
            {
            }

            const std::string getReaderInitializer(const std::string & string_id)
            {
                return (string_id);
            }

            const std::string getWriterInitializer(const std::string & string_id)
            {
                return (string_id);
            }
    };


    template <class t_Base>
    class FilenameReaderInitializer : public t_Base
    {
        public:
            FilenameReaderInitializer ()
            {
            }

            const std::string getReaderInitializer(const std::string & /*string_id*/)
            {
                return (t_Base::string_id_);
            }

            const std::string getWriterInitializer(const std::string & /*string_id*/)
            {
                return (t_Base::string_id_);
            }
    };


    class StreamInitializer
    {
        public:
            std::ifstream input_file_stream_;
            std::ofstream output_file_stream_;

        public:
            StreamInitializer()
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


#ifdef ARILES_BRIDGE_ros
    #include <ros/ros.h>

    class ROSInitializer
    {
        public:
            ros::NodeHandle *nh_;


        public:
            ROSInitializer()
            {
                int argn = 0;
                ros::init(argn, NULL, "FixtureBase");

                nh_ = new ros::NodeHandle();
            }


            ~ROSInitializer()
            {
                delete nh_;
            }

            ros::NodeHandle & getReaderInitializer(const std::string & /*string_id*/)
            {
                return (*nh_);
            }

            ros::NodeHandle & getWriterInitializer(const std::string & /*string_id*/)
            {
                return (*nh_);
            }
    };
#endif
}
