/**
    @file
    @author  Alexander Sherikov
    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles_tests
{
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


        class SizeInitializer
        {
            public:
                SizeInitializer()
                {
                }

                std::size_t getReaderInitializer(const std::string & /*string_id*/)
                {
                    return (0);
                }

                std::size_t getWriterInitializer(const std::string & /*string_id*/)
                {
                    return (0);
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
                    input_file_stream_.open(string_id.c_str());
                    ARILES_PERSISTENT_ASSERT(   true == input_file_stream_.good(),
                                            "Could not open file.");
                    return (input_file_stream_);
                }

                std::ofstream & getWriterInitializer(const std::string & string_id)
                {
                    if (true == output_file_stream_.is_open())
                    {
                        output_file_stream_.close();
                    }
                    output_file_stream_.open(string_id.c_str());
                    ARILES_PERSISTENT_ASSERT(   true == output_file_stream_.good(),
                                            "Could not open file.");
                    return (output_file_stream_);
                }
        };


#ifdef ARILES_BRIDGE_INCLUDED_ros
        #include <unistd.h>
        #include <sys/types.h>
        #include <sys/wait.h>
        #include <signal.h>
        #include <stdio.h>

        #include <ros/ros.h>

        class ROSInitializer
        {
            public:
                ros::NodeHandle *nh_;
                pid_t pid_;


            public:
                ROSInitializer()
                {
                    nh_ = NULL;
                    pid_ = fork();

                    switch (pid_)
                    {
                        case -1: // fail
                            ARILES_THROW("fork() failed");
                            break;

                        case 0: // child
                            //close(STDOUT_FILENO);
                            execlp("roscore", "roscore", (char  *) NULL);
                            ARILES_THROW("execve() failed");
                            break;

                        default: // parent
                            int argn = 0;
                            ros::init(argn, NULL, "FixtureBase");
                            while(false == ros::master::check())
                            {
                                usleep(20000);
                            }
                            nh_ = new ros::NodeHandle();
                            break;
                    }
                }


                ~ROSInitializer()
                {
                    if (NULL != nh_)
                    {
                        delete nh_;
                    }

                    if (pid_ > 0)
                    {
                        if (0 == kill(pid_, 0))
                        {
                            sighandler_t sig_handler = signal(SIGCHLD, SIG_IGN);
                            kill(pid_, SIGINT);

                            int status;
                            waitpid(pid_, &status, 0);
                            signal(SIGCHLD, sig_handler);
                        }
                    }
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
}
