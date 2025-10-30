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

            const std::string getReaderInitializer(const std::string &string_id) const
            {
                return (string_id);
            }

            const std::string getWriterInitializer(const std::string &string_id) const
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

            std::size_t getReaderInitializer(const std::string & /*string_id*/) const
            {
                return (0);
            }

            std::size_t getWriterInitializer(const std::string & /*string_id*/) const
            {
                return (0);
            }
        };


        template <class t_Base>
        class FilenameReaderInitializer : public t_Base
        {
        public:
            FilenameReaderInitializer()
            {
            }

            const std::string getReaderInitializer(const std::string & /*string_id*/) const
            {
                return (t_Base::string_id_);
            }

            const std::string getWriterInitializer(const std::string & /*string_id*/) const
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

            std::ifstream &getReaderInitializer(const std::string &string_id)
            {
                if (input_file_stream_.is_open())
                {
                    input_file_stream_.close();
                }
                input_file_stream_.open(string_id.c_str());
                CPPUT_PERSISTENT_ASSERT(input_file_stream_.good(), "Could not open file.");
                return (input_file_stream_);
            }

            std::ofstream &getWriterInitializer(const std::string &string_id)
            {
                if (output_file_stream_.is_open())
                {
                    output_file_stream_.close();
                }
                output_file_stream_.open(string_id.c_str());
                CPPUT_PERSISTENT_ASSERT(output_file_stream_.good(), "Could not open file.");
                return (output_file_stream_);
            }
        };


#ifdef ARILES2_VISITOR_INCLUDED_rosparam
#    include <unistd.h>
#    include <sys/types.h>
#    include <sys/wait.h>
#    include <signal.h>
#    include <stdio.h>

#    include <ros/ros.h>

        class ROSInitializer
        {
        private:
            ROSInitializer(const ROSInitializer &);
            void operator=(const ROSInitializer &);


        public:
            ros::NodeHandle *nh_;
            pid_t pid_;


        public:
            ROSInitializer()
            {
                nh_ = nullptr;
                pid_ = fork();

                switch (pid_)
                {
                    case -1:  // fail
                        CPPUT_THROW("fork() failed");
                        break;

                    case 0:  // child
                        // close(STDOUT_FILENO);
                        execlp("roscore", "roscore", (char *)nullptr);
                        CPPUT_THROW("execve() failed");
                        break;

                    default:  // parent
                        int argn = 0;
                        ros::init(argn, nullptr, "FixtureBase");
                        while (not ros::master::check())
                        {
                            usleep(20000);
                        }
                        nh_ = new ros::NodeHandle();
                        break;
                }
            }


            ~ROSInitializer()
            {
                if (nullptr != nh_)
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

            ros::NodeHandle &getReaderInitializer(const std::string & /*string_id*/)
            {
                return (*nh_);
            }

            ros::NodeHandle &getWriterInitializer(const std::string & /*string_id*/)
            {
                return (*nh_);
            }
        };
#endif


#ifdef ARILES2_VISITOR_INCLUDED_ros2param
#    include <rclcpp/rclcpp.hpp>


        class ROS2Initializer
        {
        private:
            inline static std::atomic<std::size_t> counter_{ 0 };

            ROS2Initializer(const ROS2Initializer &);
            void operator=(const ROS2Initializer &);


        public:
            rclcpp::Node::SharedPtr nh_;


        public:
            ROS2Initializer()
            {
                nh_ = nullptr;

                if (not rclcpp::ok())
                {
                    rclcpp::init(/*argn=*/0, /*argv=*/nullptr);
                }

                nh_ = rclcpp::Node::make_shared(
                        std::string("FixtureBase") + boost::lexical_cast<std::string>(counter_++),
                        rclcpp::NodeOptions()
                                .allow_undeclared_parameters(true)
                                .automatically_declare_parameters_from_overrides(true));
            }

            ~ROS2Initializer()
            {
                /*
                rcl_interfaces::msg::ListParametersResult parameters = nh_->list_parameters({}, 100);
                std::cout << ">>>>>>>>>" << std::endl;
                for (const std::string &name : parameters.names)
                {
                    std::cout << name << " = " << nh_->get_parameter(name).value_to_string() << std::endl;
                }
                std::cout << ">>>>>>>>>" << std::endl;
                */
                rclcpp::shutdown();
            }

            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr getReaderInitializer(
                    const std::string & /*string_id*/)
            {
                return (nh_->get_node_parameters_interface());
            }

            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr getWriterInitializer(
                    const std::string & /*string_id*/)
            {
                return (nh_->get_node_parameters_interface());
            }
        };
#endif
    }  // namespace initializers
}  // namespace ariles_tests
