/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "common.h"


namespace ariles2
{
    namespace ns_ros
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer : public ariles2::ns_ros::ImplBase
            {
            public:
                explicit Writer(const ::ros::NodeHandle &nh)
                {
                    nh_ = nh;
                }
            };
        }  // namespace impl
    }      // namespace ns_ros
}  // namespace ariles2


namespace ariles2
{
    namespace ns_ros
    {
        Writer::Writer(const ::ros::NodeHandle &nh)
        {
            impl_ = ImplPtr(new Impl(nh));
        }


        void Writer::flush()
        {
            if (XmlRpc::XmlRpcValue::TypeInvalid == impl_->root_value_.getType())
            {
                impl_->root_value_ = "";
            }
            impl_->nh_.setParam(impl_->root_name_, impl_->root_value_);
            impl_->root_name_.clear();
        }



        void Writer::startMapElement(const std::string &map_name)
        {
            if (0 == impl_->node_stack_.size())
            {
                impl_->root_name_ = map_name;
                impl_->node_stack_.push_back(&impl_->root_value_);
            }
            else
            {
                impl_->node_stack_.push_back(NodeWrapper(&(impl_->getRawNode()[map_name])));
            }
        }

        void Writer::endMapElement()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            impl_->getRawNode().setSize(size);
            impl_->node_stack_.push_back(NodeWrapper(0, size));
        }

        void Writer::startArrayElement()
        {
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
        }

        void Writer::endArrayElement()
        {
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ++impl_->node_stack_.back().index_;
        }

        void Writer::endArray()
        {
            impl_->node_stack_.pop_back();
        }



        void Writer::writeElement(const bool &element, const Parameters &)
        {
            impl_->getRawNode() = element;
        }


        void Writer::writeElement(const std::string &element, const Parameters &)
        {
            impl_->getRawNode() = element;
        }


        void Writer::startRoot(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;

            impl_->root_name_ = "";
            impl_->root_value_.clear();

            if (true == name.empty())
            {
                startMapElement("ariles");
            }
            else
            {
                startMapElement(name);
            }
        }

        void Writer::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            endMapElement();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        impl_->getRawNode() = element;                                                                                 \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_REAL_TYPES_LIST)

#undef ARILES2_BASIC_TYPE



#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        ARILES2_ASSERT(                                                                                                \
                static_cast<int64_t>(element) <= std::numeric_limits<int>::max()                                       \
                        && static_cast<int64_t>(element) >= static_cast<int64_t>(std::numeric_limits<int>::min()),     \
                "Value is out of range.");                                                                             \
        impl_->getRawNode() = static_cast<int>(element);                                                               \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        ARILES2_ASSERT(                                                                                                \
                static_cast<uint64_t>(element) <= static_cast<uint64_t>(std::numeric_limits<int>::max()),              \
                "Value is too large.");                                                                                \
        impl_->getRawNode() = static_cast<int>(element);                                                               \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_ros
}  // namespace ariles2
