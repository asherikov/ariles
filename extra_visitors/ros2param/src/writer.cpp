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


namespace ariles2
{
    namespace ns_ros2param
    {
        class WriterNodeWrapper : public serialization::Node<std::string>
        {
        protected:
            using Parent = serialization::Node<std::string>;

        public:
            enum class ArrayType
            {
                UNDEFINED = 0,
                BUILTIN = 1,
                GENERIC = 2
            };

        public:
            std::variant<
                    std::vector<uint8_t>,
                    std::vector<bool>,
                    std::vector<int64_t>,
                    std::vector<float>,
                    std::vector<double>,
                    std::vector<std::string>>
                    array_values_;
            ArrayType array_type_ = ArrayType::UNDEFINED;

        public:
            using Parent::Parent;


            WriterNodeWrapper(const std::string &name, std::size_t size) : Parent(name, 0, size)
            {
                array_type_ = ArrayType::GENERIC;
            }

            template <class t_Value>
            bool tryPushArray(const t_Value value)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(node_);
                ARILES2_TRACE_TYPE(t_Value);
                if (isArray())
                {
                    if (ArrayType::UNDEFINED == array_type_)
                    {
                        array_values_.emplace<std::vector<t_Value>>();
                        std::get<std::vector<t_Value>>(array_values_).reserve(size_);
                        array_type_ = ArrayType::BUILTIN;
                    }
                    std::get<std::vector<t_Value>>(array_values_).push_back(value);
                    return (true);
                }
                return (false);
            }

            bool tryPushArray(const std::string &value)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(node_);
                if (isArray())
                {
                    if (ArrayType::UNDEFINED == array_type_)
                    {
                        array_values_.emplace<std::vector<std::string>>();
                        std::get<std::vector<std::string>>(array_values_).reserve(size_);
                        array_type_ = ArrayType::BUILTIN;
                    }
                    std::get<std::vector<std::string>>(array_values_).push_back(value);
                    return (true);
                }
                return (false);
            }

            [[nodiscard]] bool isBuiltinArray() const
            {
                return (ArrayType::BUILTIN == array_type_);
            }

            [[nodiscard]] bool isUndefinedArray() const
            {
                return (ArrayType::UNDEFINED == array_type_);
            }
        };

        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer : public serialization::NodeStackBase<WriterNodeWrapper>
            {
            public:
                // https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html
                rclcpp::Node *nh_;

                std::vector<rclcpp::Parameter> parameters_;

                const std::string separator_ = ".";

            public:
                explicit Writer(::rclcpp::Node *nh)
                {
                    nh_ = nh;
                }


                [[nodiscard]] bool publishParameters() const
                {
                    return (nh_->set_parameters_atomically(parameters_).successful);
                }

                template <class t_Element>
                void setParameter(const t_Element element)
                {
                    ARILES2_TRACE_FUNCTION;
                    ARILES2_TRACE_VALUE(back().node_);
                    ARILES2_TRACE_TYPE(t_Element);

                    parameters_.emplace_back(back().node_, element);
                }

                void setParameter(const std::string &element)
                {
                    ARILES2_TRACE_FUNCTION;
                    ARILES2_TRACE_VALUE(back().node_);
                    parameters_.emplace_back(back().node_, element);
                }

                void setParameter()
                {
                    ARILES2_TRACE_FUNCTION;
                    if (back().isBuiltinArray())
                    {
                        std::visit([this](auto &&arg) { setParameter(arg); }, back().array_values_);
                    }
                }
            };
        }  // namespace impl
    }      // namespace ns_ros2param
}  // namespace ariles2


namespace ariles2
{
    namespace ns_ros2param
    {
        Writer::Writer(::rclcpp::Node *nh)
        {
            makeImplPtr(nh);
        }


        void Writer::flush()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(impl_->publishParameters(), "Failed to set parameters.");
        }


        void Writer::startMapEntry(const std::string &child_name)
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

        void Writer::endMapEntry()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->pop();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
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

        void Writer::startArrayElement()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(not impl_->back().isCompleted(), "Internal error: array has more elements than expected.");
        }

        void Writer::endArrayElement()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->shiftArray();
        }

        void Writer::endArray()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->setParameter();
            impl_->pop();
        }


        void Writer::writeElement(const unsigned char &element, const Parameters &)
        {
            ARILES2_TRACE_FUNCTION;
            if (not impl_->back().tryPushArray<uint8_t>(element))
            {
                impl_->setParameter(static_cast<int64_t>(element));
            }
        }


        void Writer::writeElement(const float &element, const Parameters &)
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
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
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
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
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
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
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
