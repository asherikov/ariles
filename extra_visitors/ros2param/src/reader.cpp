/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <set>

#include <boost/lexical_cast.hpp>

#include <ariles2/visitors/ros2param.h>


namespace ariles2
{
    namespace ns_ros2param
    {
        class ReaderNodeWrapper : public serialization::Node<std::string>
        {
        protected:
            using Parent = serialization::Node<std::string>;

        protected:
            std::set<std::string> childs_;
            std::set<std::string>::const_iterator childs_iterator_;
            const rclcpp::Parameter parameter_;
            const bool is_builtin_array_ = false;

        public:
            using Parent::Parent;

            ReaderNodeWrapper(const std::string &name, std::set<std::string> childs)
              : Parent(name, Parent::Type::ITERATED_MAP), childs_(std::move(childs))
            {
                size_ = childs_.size();
                childs_iterator_ = childs_.begin();
            }

            const std::string &getChildName()
            {
                return (*childs_iterator_++);
            }

            explicit ReaderNodeWrapper(const rclcpp::Parameter &&parameter)
              : Parent(Parent::Type::ARRAY), parameter_(parameter), is_builtin_array_(true)
            {
                index_ = 0;
                switch (parameter_.get_type())
                {
                    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
                        size_ = parameter_.as_byte_array().size();
                        return;
                    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
                        size_ = parameter_.as_bool_array().size();
                        return;
                    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
                        size_ = parameter_.as_integer_array().size();
                        return;
                    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
                        size_ = parameter_.as_double_array().size();
                        return;
                    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
                        size_ = parameter_.as_string_array().size();
                        return;
                    default:
                        ARILES2_THROW("Unexpected value type");
                }
            }

            bool tryReadArray(int64_t &value)
            {
                if (isBuiltinArray())
                {
                    switch (parameter_.get_type())
                    {
                        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
                            value = parameter_.as_byte_array()[index_];
                            return (true);
                        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
                            value = parameter_.as_integer_array()[index_];
                            return (true);
                        default:
                            ARILES2_THROW("Unexpected array value type");
                    }
                }
                return (false);
            }

            bool tryReadArray(double &value)
            {
                if (isBuiltinArray())
                {
                    switch (parameter_.get_type())
                    {
                        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
                            value = parameter_.as_double_array()[index_];
                            return (true);
                        default:
                            ARILES2_THROW("Unexpected array value type");
                    }
                }
                return (false);
            }

            bool tryReadArray(std::string &value)
            {
                if (isBuiltinArray())
                {
                    switch (parameter_.get_type())
                    {
                        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
                            value = parameter_.as_string_array()[index_];
                            return (true);
                        default:
                            ARILES2_THROW("Unexpected array value type");
                    }
                }
                return (false);
            }

            bool tryReadArray(bool &value)
            {
                if (isBuiltinArray())
                {
                    switch (parameter_.get_type())
                    {
                        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
                            value = parameter_.as_bool_array()[index_];
                            return (true);
                        default:
                            ARILES2_THROW("Unexpected array value type");
                    }
                }
                return (false);
            }

            [[nodiscard]] bool isNonBuiltinArray() const
            {
                return (not is_builtin_array_ and isArray());
            }

            [[nodiscard]] bool isBuiltinArray() const
            {
                return (is_builtin_array_ and isArray());
            }
        };

        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader
            {
            public:
                /// Stack of nodes.
                std::vector<ReaderNodeWrapper> node_stack_;

                // https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html
                const rclcpp::Node *nh_;

                std::vector<std::string> parameter_names_;


            public:
                explicit Reader(const ::rclcpp::Node *nh)
                {
                    nh_ = nh;
                }

                ReaderNodeWrapper &back()
                {
                    return (node_stack_.back());
                }

                [[nodiscard]] const ReaderNodeWrapper &back() const
                {
                    return (node_stack_.back());
                }

                bool getParameter(rclcpp::Parameter &parameter) const
                {
                    ARILES2_TRACE_FUNCTION;
                    ARILES2_TRACE_VALUE(back().node_);

                    return (nh_->get_parameter(back().node_, parameter));
                }


                [[nodiscard]] bool isParameter() const
                {
                    for (const std::string &name : parameter_names_)
                    {
                        if (back().node_ == name)
                        {
                            return (true);
                        }
                    }
                    return (false);
                }

                void reset()
                {
                    node_stack_.clear();

                    rcl_interfaces::msg::ListParametersResult list_msg;
                    parameter_names_ = std::move(nh_->list_parameters({}, std::numeric_limits<uint64_t>::max()).names);
                }


                [[nodiscard]] static bool isPrefix(const std::string &prefix, const std::string &name)
                {
                    if (prefix.empty())
                    {
                        return (true);  // zero prefix
                    }

                    if (prefix.size() <= name.size())
                    {
                        if (name.substr(0, prefix.size()) == prefix)
                        {
                            if (prefix.size() == name.size())
                            {
                                return (true);  // name = prefix
                            }

                            if ('.' == name[prefix.size()])
                            {
                                return (true);  // normal prefix
                            }
                        }
                    }

                    return (false);
                }


                [[nodiscard]] std::set<std::string> listParameters() const
                {
                    std::size_t substr_start = 0;

                    if (not node_stack_.empty())
                    {
                        substr_start = back().node_.size() + 1;  // + 1 for dot
                    }

                    std::set<std::string> names;
                    for (const std::string &name : parameter_names_)
                    {
                        if (isPrefix(back().node_, name))
                        {
                            const std::size_t substr_end = name.find('.', substr_start);
                            names.insert(name.substr(substr_start, substr_end - substr_start));
                        }
                    }

                    return (names);
                }

                bool hasParameterPrefix()
                {
                    for (const std::string &name : parameter_names_)
                    {
                        if (isPrefix(back().node_, name))
                        {
                            return (true);
                        }
                    }
                    return (false);
                }

                template <int t_expected_parameter_type, class t_Element>
                void readElement(t_Element &element)
                {
                    ARILES2_TRACE_FUNCTION;
                    if (not back().tryReadArray(element))
                    {
                        rclcpp::Parameter parameter;
                        ARILES2_ASSERT(getParameter(parameter), std::string("Cannot read parameter: ") + back().node_);
                        ARILES2_ASSERT(t_expected_parameter_type == parameter.get_type(), "Unexpected parameter type.");
                        element = parameter.get_value<t_Element>();
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
        Reader::Reader(const ::rclcpp::Node *nh)
        {
            makeImplPtr(nh);
        }


        bool Reader::startRoot(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_TRACE_VALUE(name);

            impl_->reset();
            return (Parent::startRoot(name));
        }


        bool Reader::startMapEntry(const std::string &child_name)
        {
            ARILES2_TRACE_FUNCTION;
            if (impl_->node_stack_.empty())
            {
                impl_->node_stack_.emplace_back(child_name);
            }
            else
            {
                ARILES2_ASSERT(not impl_->back().isBuiltinArray(), "Unexpected parent type (builtin array).");

                std::string node;
                node.reserve(impl_->back().node_.size() + child_name.size() + 1);
                node = impl_->back().node_;
                node += ".";
                node += child_name;
                impl_->node_stack_.emplace_back(std::move(node));
            }

            return (impl_->hasParameterPrefix());
        }

        void Reader::endMapEntry()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }



        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType limit_type,
                const std::size_t min,
                const std::size_t max)
        {
            ARILES2_TRACE_FUNCTION;

            std::set<std::string> name_list = impl_->listParameters();

            checkSize(limit_type, name_list.size(), min, max);

            impl_->node_stack_.emplace_back(impl_->back().node_, std::move(name_list));
            return (true);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            ARILES2_TRACE_FUNCTION;
            if (impl_->back().isCompleted())
            {
                return (false);
            }

            entry_name = impl_->back().getChildName();

            std::string node;
            node.reserve(impl_->back().node_.size() + entry_name.size() + 1);
            node = impl_->back().node_;
            node += ".";
            node += entry_name;
            impl_->node_stack_.emplace_back(std::move(node));

            return (true);
        }

        void Reader::endIteratedMapElement()
        {
            impl_->node_stack_.pop_back();
            ++(impl_->back().index_);
        }

        void Reader::endIteratedMap()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(impl_->back().isCompleted(), "End of iterated map has not been reached.");
            impl_->node_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            ARILES2_TRACE_FUNCTION;

            if (not impl_->node_stack_.empty() and impl_->isParameter())
            {
                rclcpp::Parameter values;
                impl_->getParameter(values);
                impl_->node_stack_.emplace_back(std::move(values));
            }
            else
            {
                impl_->node_stack_.emplace_back(impl_->back().node_, 0, impl_->listParameters().size());
            }

            return (impl_->back().size_);
        }

        void Reader::startArrayElement()
        {
            ARILES2_TRACE_FUNCTION;

            ARILES2_ASSERT(not impl_->back().isCompleted(), "Internal error: array has more elements than expected.");
            if (impl_->back().isNonBuiltinArray())
            {
                std::string node;
                node.reserve(impl_->back().node_.size() + num_chars_for_index_reserve + 1);
                node = impl_->back().node_;
                node += ".";
                node += boost::lexical_cast<std::string>(impl_->back().index_);

                impl_->node_stack_.emplace_back(std::move(node));
            }
        }

        void Reader::endArrayElement()
        {
            ARILES2_TRACE_FUNCTION;
            if (not impl_->back().isBuiltinArray())
            {
                impl_->node_stack_.pop_back();
            }
            ARILES2_ASSERT(impl_->back().isArray(), "Internal error: expected array.");
            ++(impl_->back().index_);
        }

        void Reader::endArray()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }



#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        int64_t tmp_value;                                                                                             \
        impl_->readElement<rclcpp::ParameterType::PARAMETER_INTEGER>(tmp_value);                                       \
        ARILES2_ASSERT(                                                                                                \
                tmp_value <= std::numeric_limits<type>::max() && tmp_value >= std::numeric_limits<type>::min(),        \
                "Value is out of range.");                                                                             \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        int64_t tmp_value;                                                                                             \
        impl_->readElement<rclcpp::ParameterType::PARAMETER_INTEGER>(tmp_value);                                       \
        ARILES2_ASSERT(tmp_value >= 0, "Expected positive value.");                                                    \
        ARILES2_ASSERT(static_cast<uint64_t>(tmp_value) <= std::numeric_limits<type>::max(), "Value is too large.");   \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        double tmp_value;                                                                                              \
        impl_->readElement<rclcpp::ParameterType::PARAMETER_DOUBLE>(tmp_value);                                        \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_REAL_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


        void Reader::readElement(std::string &element)
        {
            ARILES2_TRACE_FUNCTION;
            impl_->readElement<rclcpp::ParameterType::PARAMETER_STRING>(element);
        }


        void Reader::readElement(bool &element)
        {
            ARILES2_TRACE_FUNCTION;
            impl_->readElement<rclcpp::ParameterType::PARAMETER_BOOL>(element);
        }
    }  // namespace ns_ros2param
}  // namespace ariles2
