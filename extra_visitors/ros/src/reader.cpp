/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include <boost/lexical_cast.hpp>

#include "common.h"


namespace ariles
{
    namespace ns_ros
    {
        namespace impl
        {
            class ARILES_LIB_LOCAL Reader : public ariles::ns_ros::ImplBase
            {
            public:
                Reader(const ::ros::NodeHandle &nh)
                {
                    nh_ = nh;
                }
            };
        }  // namespace impl
    }      // namespace ns_ros
}  // namespace ariles


namespace ariles
{
    namespace ns_ros
    {
        Reader::Reader(const ::ros::NodeHandle &nh)
        {
            impl_ = ImplPtr(new Impl(nh));
        }


        std::size_t Reader::getMapSize(const bool expect_empty)
        {
            if (XmlRpc::XmlRpcValue::TypeStruct == impl_->getRawNode().getType())
            {
                return (impl_->getRawNode().size());
            }
            else
            {
                ARILES_PERSISTENT_ASSERT(true == expect_empty, "Expected struct.");
                return (0);
            }
        }


        bool Reader::descend(const std::string &child_name)
        {
            if (0 == impl_->node_stack_.size())
            {
                impl_->root_name_ = child_name;
                impl_->nh_.getParam(impl_->root_name_, impl_->root_value_);
                impl_->node_stack_.push_back(&impl_->root_value_);
                return (true);
            }
            else
            {
                XmlRpc::XmlRpcValue &node = impl_->getRawNode();
                if ((XmlRpc::XmlRpcValue::TypeStruct == node.getType()) && (true == node.hasMember(child_name)))
                {
                    impl_->node_stack_.push_back(NodeWrapper(&(node[child_name])));
                    return (true);
                }
                else
                {
                    return (false);
                }
            }
        }


        void Reader::ascend()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::getMapEntryNames(std::vector<std::string> &child_names)
        {
            XmlRpc::XmlRpcValue selected_node = impl_->getRawNode();

            if (XmlRpc::XmlRpcValue::TypeStruct != selected_node.getType())
            {
                return (false);
            }
            else
            {
                child_names.resize(selected_node.size());

                std::size_t i = 0;
                for (XmlRpc::XmlRpcValue::iterator it = selected_node.begin(); it != selected_node.end(); ++it, ++i)
                {
                    child_names[i] = it->first;
                }
                return (true);
            }
        }


        std::size_t Reader::startArray()
        {
            ARILES_ASSERT(XmlRpc::XmlRpcValue::TypeArray == impl_->getRawNode().getType(), "Expected array.");

            std::size_t size = impl_->getRawNode().size();
            impl_->node_stack_.push_back(NodeWrapper(0, size));

            return (size);
        }


        void Reader::shiftArray()
        {
            ARILES_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ARILES_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
            ++impl_->node_stack_.back().index_;
        }


        void Reader::endArray()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::startRoot(const std::string &name)
        {
            ARILES_TRACE_FUNCTION;
            if (true == name.empty())
            {
                return (descend("ariles"));
            }
            else
            {
                return (descend(name));
            }
        }

        void Reader::endRoot(const std::string & /*name*/)
        {
            ARILES_TRACE_FUNCTION;
            ascend();
        }


#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        ARILES_ASSERT(impl_->getRawNode().getType() == XmlRpc::XmlRpcValue::TypeInt, "Integer type expected.");        \
        int tmp_value = static_cast<int>(impl_->getRawNode());                                                         \
        ARILES_ASSERT(                                                                                                 \
                tmp_value <= std::numeric_limits<type>::max() && tmp_value >= std::numeric_limits<type>::min(),        \
                "Value is out of range.");                                                                             \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES_BASIC_TYPE


#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        ARILES_ASSERT(impl_->getRawNode().getType() == XmlRpc::XmlRpcValue::TypeInt, "Integer type expected.");        \
        int tmp_value = static_cast<int>(impl_->getRawNode());                                                         \
        ARILES_ASSERT(tmp_value >= 0, "Expected positive value.");                                                     \
        ARILES_ASSERT(                                                                                                 \
                static_cast<unsigned int>(tmp_value) <= std::numeric_limits<type>::max(), "Value is too large.");      \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES_BASIC_TYPE


#define ARILES_BASIC_TYPE(type)                                                                                        \
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
                ARILES_THROW("Could not convert value to type.");                                                      \
                break;                                                                                                 \
        }                                                                                                              \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_REAL_TYPES_LIST)

#undef ARILES_BASIC_TYPE


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
                    ARILES_THROW("Could not convert value to boolean.");
                    break;
            }
        }
    }  // namespace ns_ros
}  // namespace ariles
