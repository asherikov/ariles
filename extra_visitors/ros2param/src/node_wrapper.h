/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

namespace ariles2
{
    namespace ns_ros2param
    {
        class NodeBase : public serialization::Node<std::string>
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
            ArrayType array_type_ = ArrayType::UNDEFINED;

        public:
            using Parent::Parent;

            [[nodiscard]] bool isNonBuiltinArray() const
            {
                return (ArrayType::BUILTIN != array_type_ and isArray());
            }

            [[nodiscard]] bool isBuiltinArray() const
            {
                return (ArrayType::BUILTIN == array_type_ and isArray());
            }
        };


        class ModifierNode : public NodeBase
        {
        public:
            std::variant<
                    std::vector<uint8_t>,
                    std::vector<bool>,
                    std::vector<int64_t>,
                    std::vector<float>,
                    std::vector<double>,
                    std::vector<std::string>>
                    array_values_;

        public:
            using NodeBase::NodeBase;


            ModifierNode(const std::string &name, std::size_t size) : NodeBase(name, 0, size)
            {
                array_type_ = ArrayType::GENERIC;
            }

            template <class t_Value>
            bool tryPushArray(const t_Value value)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(node_);
                CPPUT_TRACE_TYPE(t_Value);
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
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(node_);
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
        };
    }  // namespace ns_ros2param
}  // namespace ariles2
