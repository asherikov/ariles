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

#include "node_wrapper.h"


namespace ariles2
{
    namespace ns_ros2param
    {
        class ReaderNodeWrapper : public NodeBase
        {
        protected:
            std::set<std::string> childs_;
            std::set<std::string>::const_iterator childs_iterator_;
            const rclcpp::Parameter parameter_;

        public:
            using NodeBase::NodeBase;

            ReaderNodeWrapper(const std::string &name, std::set<std::string> childs)
              : NodeBase(name, NodeBase::Type::ITERATED_MAP), childs_(std::move(childs))
            {
                size_ = childs_.size();
                childs_iterator_ = childs_.begin();
            }

            const std::string &getChildName()
            {
                return (*childs_iterator_++);
            }

            explicit ReaderNodeWrapper(const rclcpp::Parameter &&parameter)
              : NodeBase(NodeBase::Type::ARRAY), parameter_(parameter)
            {
                index_ = 0;
                array_type_ = ArrayType::BUILTIN;

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
                        CPPUT_THROW("Unexpected value type");
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
                            CPPUT_THROW("Unexpected array value type");
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
                            CPPUT_THROW("Unexpected array value type");
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
                            CPPUT_THROW("Unexpected array value type");
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
                            CPPUT_THROW("Unexpected array value type");
                    }
                }
                return (false);
            }
        };

        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader : public serialization::NodeStackBase<ReaderNodeWrapper>
            {
            public:
                // https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html
                rclcpp::node_interfaces::NodeParametersInterface::SharedPtr nh_;

                std::vector<std::string> parameter_names_;

                const std::string separator_ = ".";


            public:
                explicit Reader(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &nh)
                {
                    nh_ = nh;
                }


                bool getParameter(rclcpp::Parameter &parameter) const
                {
                    CPPUT_TRACE_FUNCTION;
                    CPPUT_TRACE_VALUE(back().node_);

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
                    clear();

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

                    if (not empty())
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
                    CPPUT_TRACE_FUNCTION;
                    if (not back().tryReadArray(element))
                    {
                        rclcpp::Parameter parameter;
                        CPPUT_ASSERT(getParameter(parameter), std::string("Cannot read parameter: ") + back().node_);
                        CPPUT_ASSERT(t_expected_parameter_type == parameter.get_type(), "Unexpected parameter type.");
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
        Reader::Reader(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &nh)
        {
            makeImplPtr(nh);
        }


        bool Reader::startRoot(const std::string &name)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_TRACE_VALUE(name);

            impl_->reset();
            return (Parent::startRoot(name));
        }


        bool Reader::startMapEntry(const std::string &child_name)
        {
            CPPUT_TRACE_FUNCTION;
            if (impl_->empty())
            {
                impl_->emplace(child_name);
            }
            else
            {
                CPPUT_ASSERT(not impl_->back().isBuiltinArray(), "Unexpected parent type (builtin array).");

                impl_->concatWithNodeAndEmplace(impl_->separator_, child_name);
            }

            return (impl_->hasParameterPrefix());
        }

        void Reader::endMapEntry()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
        }



        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType limit_type,
                const std::size_t min,
                const std::size_t max)
        {
            CPPUT_TRACE_FUNCTION;

            std::set<std::string> name_list = impl_->listParameters();

            checkSize(limit_type, name_list.size(), min, max);

            impl_->emplace(impl_->back().node_, std::move(name_list));
            return (true);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            CPPUT_TRACE_FUNCTION;
            if (impl_->back().isCompleted())
            {
                return (false);
            }

            entry_name = impl_->back().getChildName();
            impl_->concatWithNodeAndEmplace(impl_->separator_, entry_name);

            return (true);
        }

        void Reader::endIteratedMapElement()
        {
            impl_->pop();
            ++(impl_->back().index_);
        }

        void Reader::endIteratedMap()
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(impl_->back().isCompleted(), "End of iterated map has not been reached.");
            impl_->pop();
        }


        std::size_t Reader::startArray()
        {
            CPPUT_TRACE_FUNCTION;

            if (not impl_->empty() and impl_->isParameter())
            {
                rclcpp::Parameter values;
                impl_->getParameter(values);
                impl_->emplace(std::move(values));
            }
            else
            {
                impl_->emplace(impl_->back().node_, 0, impl_->listParameters().size());
            }

            return (impl_->back().size_);
        }

        void Reader::startArrayElement()
        {
            CPPUT_TRACE_FUNCTION;

            CPPUT_ASSERT(not impl_->back().isCompleted(), "Internal error: array has more elements than expected.");
            if (impl_->back().isNonBuiltinArray())
            {
                impl_->concatWithNodeAndEmplace(
                        impl_->separator_, boost::lexical_cast<std::string>(impl_->back().index_));
            }
        }

        void Reader::endArrayElement()
        {
            CPPUT_TRACE_FUNCTION;
            if (not impl_->back().isBuiltinArray())
            {
                impl_->pop();
            }
            impl_->shiftArray();
        }

        void Reader::endArray()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
        }



#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        int64_t tmp_value;                                                                                             \
        impl_->readElement<rclcpp::ParameterType::PARAMETER_INTEGER>(tmp_value);                                       \
        CPPUT_ASSERT(                                                                                                  \
                tmp_value <= std::numeric_limits<type>::max() && tmp_value >= std::numeric_limits<type>::min(),        \
                "Value is out of range.");                                                                             \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        int64_t tmp_value;                                                                                             \
        impl_->readElement<rclcpp::ParameterType::PARAMETER_INTEGER>(tmp_value);                                       \
        CPPUT_ASSERT(tmp_value >= 0, "Expected positive value.");                                                      \
        CPPUT_ASSERT(static_cast<uint64_t>(tmp_value) <= std::numeric_limits<type>::max(), "Value is too large.");     \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        double tmp_value;                                                                                              \
        impl_->readElement<rclcpp::ParameterType::PARAMETER_DOUBLE>(tmp_value);                                        \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_REAL_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


        void Reader::readElement(std::string &element)
        {
            CPPUT_TRACE_FUNCTION;
            impl_->readElement<rclcpp::ParameterType::PARAMETER_STRING>(element);
        }


        void Reader::readElement(bool &element)
        {
            CPPUT_TRACE_FUNCTION;
            impl_->readElement<rclcpp::ParameterType::PARAMETER_BOOL>(element);
        }
    }  // namespace ns_ros2param
}  // namespace ariles2
