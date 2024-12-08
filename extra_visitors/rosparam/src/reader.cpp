/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include <boost/lexical_cast.hpp>

#include "common.h"


namespace ariles2
{
    namespace ns_rosparam
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_PUBLIC Reader : public ariles2::ns_rosparam::ImplBase
            {
            public:
                std::vector<XmlRpc::XmlRpcValue::iterator> iterator_stack_;

            public:
                explicit Reader(const ::ros::NodeHandle &nh)
                {
                    nh_ = nh;
                }
            };
        }  // namespace impl
    }  // namespace ns_rosparam
}  // namespace ariles2


namespace ariles2
{
    namespace ns_rosparam
    {
        Reader::Reader(const ::ros::NodeHandle &nh)
        {
            makeImplPtr(nh);
        }


        void Reader::startMap(const SizeLimitEnforcementType limit_type, const std::size_t min, const std::size_t max)
        {
            CPPUT_TRACE_FUNCTION;
            if (XmlRpc::XmlRpcValue::TypeStruct == impl_->getRawNode().getType())
            {
                checkSize(limit_type, impl_->getRawNode().size(), min, max);
            }
            else
            {
                CPPUT_PERSISTENT_ASSERT(SIZE_LIMIT_NONE == limit_type or (0 == min and min == max), "Expected struct.");
            }
        }

        bool Reader::startMapEntry(const std::string &child_name)
        {
            if (impl_->empty())
            {
                impl_->root_name_ = child_name;
                impl_->nh_.getParam(impl_->root_name_, impl_->root_value_);
                impl_->emplace(&impl_->root_value_);
                return (true);
            }

            XmlRpc::XmlRpcValue &node = impl_->getRawNode();
            if ((XmlRpc::XmlRpcValue::TypeStruct == node.getType()) && (node.hasMember(child_name)))
            {
                impl_->emplace(&(node[child_name]));
                return (true);
            }
            return (false);
        }

        void Reader::endMapEntry()
        {
            impl_->pop();
        }



        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType limit_type,
                const std::size_t min,
                const std::size_t max)
        {
            CPPUT_TRACE_FUNCTION;
            if (XmlRpc::XmlRpcValue::TypeStruct == impl_->getRawNode().getType())
            {
                checkSize(limit_type, impl_->getRawNode().size(), min, max);
                impl_->iterator_stack_.push_back(impl_->getRawNode().begin());
                return (true);
            }
            CPPUT_PERSISTENT_ASSERT(0 == min and min == max, "Expected struct.");
            return (false);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            if (impl_->iterator_stack_.back() != impl_->getRawNode().end())
            {
                impl_->emplace(&impl_->iterator_stack_.back()->second);
                entry_name = impl_->iterator_stack_.back()->first;
                return (true);
            }
            return (false);
        }

        void Reader::endIteratedMapElement()
        {
            ++impl_->iterator_stack_.back();
            impl_->pop();
        }

        void Reader::endIteratedMap()
        {
            CPPUT_ASSERT(
                    impl_->iterator_stack_.back() == impl_->getRawNode().end(),
                    "End of iterated map has not been reached.");
            impl_->iterator_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            CPPUT_ASSERT(XmlRpc::XmlRpcValue::TypeArray == impl_->getRawNode().getType(), "Expected array.");

            std::size_t size = impl_->getRawNode().size();
            impl_->emplace(nullptr, 0, size);

            return (size);
        }

        void Reader::startArrayElement()
        {
            CPPUT_ASSERT(
                    impl_->back().index_ < impl_->back().size_,
                    "Internal error: array has more elements than expected.");
        }

        void Reader::endArrayElement()
        {
            impl_->shiftArray();
        }

        void Reader::endArray()
        {
            impl_->pop();
        }


        bool Reader::startRoot(const std::string &name)
        {
            CPPUT_TRACE_FUNCTION;
            if (name.empty())
            {
                return (startMapEntry("ariles"));
            }
            return (startMapEntry(name));
        }

        void Reader::endRoot(const std::string & /*name*/)
        {
            CPPUT_TRACE_FUNCTION;
            endMapEntry();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        CPPUT_ASSERT(impl_->getRawNode().getType() == XmlRpc::XmlRpcValue::TypeInt, "Integer type expected.");         \
        int tmp_value = static_cast<int>(impl_->getRawNode());                                                         \
        CPPUT_ASSERT(                                                                                                  \
                static_cast<int64_t>(tmp_value) <= std::numeric_limits<type>::max()                                    \
                        && static_cast<int64_t>(tmp_value) >= std::numeric_limits<type>::min(),                        \
                "Value is out of range.");                                                                             \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        CPPUT_ASSERT(impl_->getRawNode().getType() == XmlRpc::XmlRpcValue::TypeInt, "Integer type expected.");         \
        int tmp_value = static_cast<int>(impl_->getRawNode());                                                         \
        CPPUT_ASSERT(tmp_value >= 0, "Expected positive value.");                                                      \
        CPPUT_ASSERT(static_cast<uint64_t>(tmp_value) <= std::numeric_limits<type>::max(), "Value is too large.");     \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        switch (impl_->getRawNode().getType())                                                                         \
        {                                                                                                              \
            case XmlRpc::XmlRpcValue::TypeDouble:                                                                      \
                element = static_cast<double>(impl_->getRawNode());                                                    \
                break;                                                                                                 \
            case XmlRpc::XmlRpcValue::TypeString:                                                                      \
                element = boost::lexical_cast<double>(static_cast<std::string>(impl_->getRawNode()));                  \
                break;                                                                                                 \
            case XmlRpc::XmlRpcValue::TypeInt:                                                                         \
                element = static_cast<int>(impl_->getRawNode());                                                       \
                break;                                                                                                 \
            default:                                                                                                   \
                CPPUT_THROW("Could not convert value to type.");                                                       \
                break;                                                                                                 \
        }                                                                                                              \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_REAL_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


        void Reader::readElement(std::string &element)
        {
            element = static_cast<std::string>(impl_->getRawNode());
        }


        void Reader::readElement(bool &element)
        {
            switch (impl_->getRawNode().getType())
            {
                case XmlRpc::XmlRpcValue::TypeString:
                    element = boost::lexical_cast<bool>(static_cast<std::string>(impl_->getRawNode()));
                    break;

                case XmlRpc::XmlRpcValue::TypeBoolean:
                    element = static_cast<bool>(impl_->getRawNode());
                    break;

                case XmlRpc::XmlRpcValue::TypeInt:
                    element = static_cast<int>(impl_->getRawNode()) > 0;
                    break;

                default:
                    CPPUT_THROW("Could not convert value to boolean.");
                    break;
            }
        }
    }  // namespace ns_rosparam
}  // namespace ariles2
