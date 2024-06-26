/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles2
{
    namespace ns_ros2param
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Declarator;
        }



        /**
         * @brief Parameter declarator
         */
        class ARILES2_VISIBILITY_ATTRIBUTE Declarator
          : public serialization::PIMPLVisitor<write::Visitor, impl::Declarator>
        {
        public:
            explicit Declarator(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &nh);


            void flush();


            void startMapEntry(const std::string &child_name);
            void endMapEntry();

            void startArray(const std::size_t size, const bool /*compact*/ = false);
            void startArrayElement();
            void endArrayElement();
            void endArray();


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element, const Parameters &param);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_ros2param
}  // namespace ariles2
