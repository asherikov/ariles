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
    namespace ns_rosparam
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer : public ariles2::ns_rosparam::ImplBase
            {
            public:
                explicit Writer(const ::ros::NodeHandle &nh)
                {
                    nh_ = nh;
                }
            };
        }  // namespace impl
    }      // namespace ns_rosparam
}  // namespace ariles2


namespace ariles2
{
    namespace ns_rosparam
    {
        Writer::Writer(const ::ros::NodeHandle &nh)
        {
            makeImplPtr(nh);
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



        void Writer::startMapEntry(const std::string &map_name)
        {
            if (impl_->node_stack_.empty())
            {
                impl_->root_name_ = map_name;
                impl_->node_stack_.emplace_back(&impl_->root_value_);
            }
            else
            {
                impl_->node_stack_.emplace_back(&(impl_->getRawNode()[map_name]));
            }
        }

        void Writer::endMapEntry()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            impl_->getRawNode().setSize(static_cast<int>(size));
            impl_->node_stack_.emplace_back(0, size);
        }

        void Writer::startArrayElement()
        {
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: namevalue.has more elements than expected.");
        }

        void Writer::endArrayElement()
        {
            ARILES2_ASSERT(impl_->node_stack_.back().isArray(), "Internal error: expected array.");
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


        void Writer::startRoot(const std::string &name, const Parameters &)
        {
            ARILES2_TRACE_FUNCTION;

            impl_->root_name_ = "";
            impl_->root_value_.clear();

            if (name.empty())
            {
                startMapEntry("ariles");
            }
            else
            {
                startMapEntry(name);
            }
        }

        void Writer::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            endMapEntry();
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
    }  // namespace ns_rosparam
}  // namespace ariles2
