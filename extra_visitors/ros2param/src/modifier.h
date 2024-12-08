/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors_impl/serialization.h>
#include "node_wrapper.h"


namespace ariles2
{
    namespace ns_ros2param
    {
        class ARILES2_VISIBILITY_PUBLIC ModifierImplBase : public serialization::NodeStackBase<ModifierNode>
        {
        public:
            // https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html
            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr nh_;

            std::vector<rclcpp::Parameter> parameters_;

            const std::string separator_ = ".";

        public:
            explicit ModifierImplBase(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &nh)
            {
                nh_ = nh;
            }


            template <class t_Element>
            void setParameter(const t_Element element)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(back().node_);
                CPPUT_TRACE_TYPE(t_Element);

                parameters_.emplace_back(back().node_, element);
            }

            void setParameter(const std::string &element)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(back().node_);
                parameters_.emplace_back(back().node_, element);
            }

            void setParameter()
            {
                CPPUT_TRACE_FUNCTION;
                if (back().isBuiltinArray())
                {
                    std::visit([this](auto &&arg) { setParameter(arg); }, back().array_values_);
                }
            }
        };
    }  // namespace ns_ros2param
}  // namespace ariles2
