/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include <variant>
#include <boost/lexical_cast.hpp>

#include <ariles2/visitors/ros2param.h>

#include "modifier.h"


namespace ariles2
{
    namespace ns_ros2param
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Declarator : public ModifierImplBase
            {
            public:
                using ModifierImplBase::ModifierImplBase;

                [[nodiscard]] bool publishParameters() const
                {
                    for (const rclcpp::Parameter &parameter : parameters_)
                    {
                        if (not nh_->has_parameter(parameter.get_name()))
                        {
                            const rclcpp::ParameterValue &declared_value =
                                    nh_->declare_parameter(parameter.get_name(), parameter.get_parameter_value());

                            ARILES2_ASSERT(
                                    declared_value.get_type() == parameter.get_type(),
                                    std::string("Parameter type mismatch: ") + parameter.get_name());
                        }

                        // https://github.com/ros2/rclcpp/blob/master/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp#L652
                        // "cannot undeclare a statically typed parameter"
                        // nh_->undeclare_parameter(parameter.get_name());
                    }
                    return (true);
                }
            };
        }  // namespace impl
    }      // namespace ns_ros2param
}  // namespace ariles2


namespace ariles2
{
    namespace ns_ros2param
    {
        Declarator::Declarator(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &nh)
        {
            makeImplPtr(nh);
        }


        void Declarator::flush()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(impl_->publishParameters(), "Failed to set parameters.");
        }


        void Declarator::startMapEntry(const std::string &child_name)
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_TRACE_VALUE(child_name);
            if (impl_->empty())
            {
                impl_->emplace(child_name);
            }
            else
            {
                if (impl_->back().isArray())
                {
                    impl_->concatWithNodeAndEmplace(
                            impl_->separator_,
                            boost::lexical_cast<std::string>(impl_->back().index_),
                            impl_->separator_,
                            child_name);
                }
                else
                {
                    impl_->concatWithNodeAndEmplace(impl_->separator_, child_name);
                }
            }
        }

        void Declarator::endMapEntry()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->pop();
        }


        void Declarator::startArray(const std::size_t size, const bool /*compact*/)
        {
            ARILES2_TRACE_FUNCTION;
            if (impl_->back().isArray())
            {
                impl_->emplace(
                        impl_->concatWithNode(
                                impl_->separator_, boost::lexical_cast<std::string>(impl_->back().index_)),
                        /*index=*/0,
                        size);
            }
            else
            {
                impl_->emplace(impl_->back().node_, /*index=*/0, size);
            }
        }

        void Declarator::startArrayElement()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(not impl_->back().isCompleted(), "Internal error: array has more elements than expected.");
        }

        void Declarator::endArrayElement()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->shiftArray();
        }

        void Declarator::endArray()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->setParameter();
            impl_->pop();
        }


        void Declarator::writeElement(const unsigned char &element, const Parameters &)
        {
            ARILES2_TRACE_FUNCTION;
            if (not impl_->back().tryPushArray<uint8_t>(element))
            {
                impl_->setParameter(static_cast<int64_t>(element));
            }
        }


        void Declarator::writeElement(const float &element, const Parameters &)
        {
            ARILES2_TRACE_FUNCTION;
            if (not impl_->back().tryPushArray<double>(element))
            {
                impl_->setParameter(static_cast<double>(element));
            }
        }


#define ARILES2_ROS2PARAM_NATIVE_TYPES_LIST                                                                            \
    ARILES2_BASIC_TYPE(bool)                                                                                           \
    ARILES2_BASIC_TYPE(double)                                                                                         \
    ARILES2_BASIC_TYPE(std::string)


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Declarator::writeElement(const type &element, const Parameters &)                                             \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        if (not impl_->back().tryPushArray(element))                                                                   \
        {                                                                                                              \
            impl_->setParameter(element);                                                                              \
        }                                                                                                              \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_ROS2PARAM_NATIVE_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Declarator::writeElement(const type &element, const Parameters &)                                             \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        if (not impl_->back().tryPushArray<int64_t>(element))                                                          \
        {                                                                                                              \
            impl_->setParameter(static_cast<int64_t>(element));                                                        \
        }                                                                                                              \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Declarator::writeElement(const type &element, const Parameters &)                                             \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_ASSERT(                                                                                                \
                static_cast<uint64_t>(element) <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max()),          \
                "Value is too large.");                                                                                \
        if (not impl_->back().tryPushArray<int64_t>(element))                                                          \
        {                                                                                                              \
            impl_->setParameter(static_cast<int64_t>(element));                                                        \
        }                                                                                                              \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST_WITHOUT_BYTE)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_ros2param
}  // namespace ariles2
