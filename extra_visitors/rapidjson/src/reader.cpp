/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <boost/math/special_functions.hpp>
#include <ariles2/visitors/rapidjson.h>
#include "common.h"


namespace ariles2
{
    namespace ns_rapidjson
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader : public ariles2::ns_rapidjson::ImplBase<const ::rapidjson::Value>
            {
            public:
                std::vector< ::rapidjson::Value::ConstMemberIterator> iterator_stack_;

            public:
                void initialize(std::istream &input_stream)
                {
                    ariles2::ns_rapidjson::IStreamWrapper isw(input_stream);
                    document_.ParseStream(isw);
                    ARILES2_ASSERT(false == document_.HasParseError(), "Parsing failed");
                }
            };
        }  // namespace impl
    }      // namespace ns_rapidjson
}  // namespace ariles2



namespace ariles2
{
    namespace ns_rapidjson
    {
        Reader::Reader(const std::string &file_name)
        {
            std::ifstream config_ifs;
            read::Visitor::openFile(config_ifs, file_name);
            impl_ = ImplPtr(new impl::Reader());
            impl_->initialize(config_ifs);
        }


        Reader::Reader(std::istream &input_stream)
        {
            impl_ = ImplPtr(new impl::Reader());
            impl_->initialize(input_stream);
        }


        void Reader::constructFromString(const char *input_string)
        {
            impl_ = ImplPtr(new impl::Reader());
            impl_->document_.Parse(input_string);
        }


        void Reader::startMap(const SizeLimitEnforcementType limit_type, const std::size_t min, const std::size_t max)
        {
            ARILES2_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().MemberCount(), min, max);
        }

        bool Reader::startMapEntry(const std::string &child_name)
        {
            const ::rapidjson::Value::ConstMemberIterator child = impl_->getRawNode().FindMember(child_name.c_str());

            if (impl_->getRawNode().MemberEnd() == child)
            {
                return (false);
            }
            impl_->node_stack_.push_back(impl::Reader::NodeWrapper(&(child->value)));
            return (true);
        }

        void Reader::endMapEntry()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType limit_type,
                const std::size_t min,
                const std::size_t max)
        {
            ARILES2_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().MemberCount(), min, max);


            const ::rapidjson::Value &selected_node = impl_->getRawNode();

            if (true == selected_node.IsObject())
            {
                impl_->iterator_stack_.push_back(selected_node.MemberBegin());
                return (true);
            }
            return (false);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            if (impl_->iterator_stack_.back() != impl_->getRawNode().MemberEnd())
            {
                impl_->node_stack_.push_back(impl::Reader::NodeWrapper(&(impl_->iterator_stack_.back()->value)));
                entry_name = impl_->iterator_stack_.back()->name.GetString();
                return (true);
            }
            return (false);
        }

        void Reader::endIteratedMapElement()
        {
            ++impl_->iterator_stack_.back();
            impl_->node_stack_.pop_back();
        }

        void Reader::endIteratedMap()
        {
            ARILES2_ASSERT(
                    impl_->iterator_stack_.back() == impl_->getRawNode().MemberEnd(),
                    "End of iterated map has not been reached.");
            impl_->iterator_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            ARILES2_ASSERT(impl_->getRawNode().IsArray(), "Internal error: expected array.");

            std::size_t size = impl_->getRawNode().Size();
            impl_->node_stack_.push_back(impl::Reader::NodeWrapper(0, size));

            return (size);
        }


        void Reader::startArrayElement()
        {
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: namevalue.has more elements than expected.");
        }


        void Reader::endArrayElement()
        {
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ++impl_->node_stack_.back().index_;
        }


        void Reader::endArray()
        {
            impl_->node_stack_.pop_back();
        }


        void Reader::readElement(std::string &element)
        {
            element = impl_->getRawNode().GetString();
        }


        void Reader::readElement(bool &element)
        {
            element = impl_->getRawNode().GetBool();
        }


        void Reader::readElement(float &element)
        {
            float tmp_value = 0.0;
            if (true == impl_->getRawNode().IsString())
            {
                tmp_value = boost::lexical_cast<float>(impl_->getRawNode().GetString());
                if (true == boost::math::isnan(tmp_value))
                {
                    element = std::numeric_limits<float>::signaling_NaN();
                    return;
                }
                if (true == boost::math::isinf(tmp_value))
                {
                    element = static_cast<float>(tmp_value);
                    return;
                }
            }
            else
            {
                tmp_value = static_cast<float>(impl_->getRawNode().GetDouble());  // old API compatibility
                // tmp_value = impl_->getRawNode().GetFloat();
            }
            ARILES2_ASSERT(
                    tmp_value <= std::numeric_limits<float>::max() && tmp_value >= -std::numeric_limits<float>::max(),
                    "Value is out of range.");
            element = static_cast<float>(tmp_value);
        }


        void Reader::readElement(double &element)
        {
            double tmp_value = 0.0;
            if (true == impl_->getRawNode().IsString())
            {
                tmp_value = boost::lexical_cast<double>(impl_->getRawNode().GetString());
                if (true == boost::math::isnan(tmp_value))
                {
                    element = std::numeric_limits<double>::signaling_NaN();
                    return;
                }
                if (true == boost::math::isinf(tmp_value))
                {
                    element = static_cast<double>(tmp_value);
                    return;
                }
            }
            else
            {
                tmp_value = impl_->getRawNode().GetDouble();
            }
            ARILES2_ASSERT(
                    tmp_value <= std::numeric_limits<double>::max() && tmp_value >= -std::numeric_limits<double>::max(),
                    "Value is out of range.");
            element = static_cast<double>(tmp_value);
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        int64_t tmp_value = impl_->getRawNode().GetInt64();                                                            \
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
        uint64_t tmp_value = impl_->getRawNode().GetUint64();                                                          \
        ARILES2_ASSERT(tmp_value <= std::numeric_limits<type>::max(), "Value is too large.");                          \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_rapidjson
}  // namespace ariles2
