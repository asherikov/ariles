/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles/visitors/rapidjson.h>
#include "common.h"


namespace ariles
{
    namespace ns_rapidjson
    {
        namespace impl
        {
            class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ns_rapidjson::ImplBase<const ::rapidjson::Value>
            {
            public:
                void initialize(std::istream &input_stream)
                {
                    ariles::ns_rapidjson::IStreamWrapper isw(input_stream);
                    document_.ParseStream(isw);
                    ARILES_ASSERT(false == document_.HasParseError(), "Parsing failed");
                }
            };
        }  // namespace impl
    }      // namespace ns_rapidjson
}  // namespace ariles



namespace ariles
{
    namespace ns_rapidjson
    {
        Reader::Reader(const std::string &file_name, const Flags &flags) : Base(flags)
        {
            std::ifstream config_ifs;
            read::Visitor::openFile(config_ifs, file_name);
            impl_ = ImplPtr(new Impl());
            impl_->initialize(config_ifs);
        }


        Reader::Reader(std::istream &input_stream, const Flags &flags) : Base(flags)
        {
            impl_ = ImplPtr(new Impl());
            impl_->initialize(input_stream);
        }


        void Reader::constructFromString(const char *input_string)
        {
            impl_ = ImplPtr(new Impl());
            impl_->document_.Parse(input_string);
        }


        std::size_t Reader::getMapSize(const bool /*expect_empty*/)
        {
            return (impl_->getRawNode().MemberCount());
        }



        bool Reader::descend(const std::string &child_name)
        {
            const ::rapidjson::Value::ConstMemberIterator child = impl_->getRawNode().FindMember(child_name.c_str());

            if (impl_->getRawNode().MemberEnd() == child)
            {
                return (false);
            }
            else
            {
                impl_->node_stack_.push_back(impl::Reader::NodeWrapper(&(child->value)));
                return (true);
            }
        }


        void Reader::ascend()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::getMapEntryNames(std::vector<std::string> &child_names)
        {
            const ::rapidjson::Value &selected_node = impl_->getRawNode();

            if (false == selected_node.IsObject())
            {
                return (false);
            }
            else
            {
                child_names.resize(selected_node.MemberCount());

                std::size_t i = 0;
                for (::rapidjson::Value::ConstMemberIterator it = selected_node.MemberBegin();
                     it != selected_node.MemberEnd();
                     ++it, ++i)
                {
                    child_names[i] = it->name.GetString();
                }
                return (true);
            }
        }


        std::size_t Reader::startArray()
        {
            std::size_t size = impl_->getRawNode().Size();
            impl_->node_stack_.push_back(impl::Reader::NodeWrapper(0, size));

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
            float tmp_value;
            if (true == impl_->getRawNode().IsString())
            {
                tmp_value = boost::lexical_cast<float>(impl_->getRawNode().GetString());
                if (true == ariles::isNaN(tmp_value))
                {
                    element = std::numeric_limits<float>::signaling_NaN();
                    return;
                }
                if (true == ariles::isInfinity(tmp_value))
                {
                    element = static_cast<float>(tmp_value);
                    return;
                }
            }
            else
            {
                tmp_value = impl_->getRawNode().GetDouble();  // old API compatibility
                // tmp_value = impl_->getRawNode().GetFloat();
            }
            ARILES_ASSERT(
                    tmp_value <= std::numeric_limits<float>::max() && tmp_value >= -std::numeric_limits<float>::max(),
                    "Value is out of range.");
            element = static_cast<float>(tmp_value);
        }


        void Reader::readElement(double &element)
        {
            double tmp_value;
            if (true == impl_->getRawNode().IsString())
            {
                tmp_value = boost::lexical_cast<double>(impl_->getRawNode().GetString());
                if (true == ariles::isNaN(tmp_value))
                {
                    element = std::numeric_limits<double>::signaling_NaN();
                    return;
                }
                if (true == ariles::isInfinity(tmp_value))
                {
                    element = static_cast<double>(tmp_value);
                    return;
                }
            }
            else
            {
                tmp_value = impl_->getRawNode().GetDouble();
            }
            ARILES_ASSERT(
                    tmp_value <= std::numeric_limits<double>::max() && tmp_value >= -std::numeric_limits<double>::max(),
                    "Value is out of range.");
            element = static_cast<double>(tmp_value);
        }


#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        int64_t tmp_value = impl_->getRawNode().GetInt64();                                                            \
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
        uint64_t tmp_value = impl_->getRawNode().GetUint64();                                                          \
        ARILES_ASSERT(tmp_value <= std::numeric_limits<type>::max(), "Value is too large.");                           \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES_BASIC_TYPE
    }  // namespace ns_rapidjson
}  // namespace ariles
