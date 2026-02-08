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
                        std::string("FixtureBase") + boost::lexical_cast<std::string>(counter_++));
            }

            ~ROS2Initializer()
            {
                rclcpp::shutdown();
            }

            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr getDeclaratorInitializer(
                    const std::string & /*string_id*/)
            {
                return (nh_->get_node_parameters_interface());
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
