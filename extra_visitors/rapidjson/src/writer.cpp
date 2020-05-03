/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include <ariles/visitors/rapidjson.h>

#include "common.h"

#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>


namespace ariles
{
    namespace ns_rapidjson
    {
        namespace impl
        {
            class ARILES_LIB_LOCAL Writer : public ariles::ns_rapidjson::ImplBase< ::rapidjson::Value>
            {
            public:
                /// output file stream
                std::ofstream config_ofs_;

                /// output stream
                std::ostream *output_stream_;

            public:
                explicit Writer(const std::string &file_name)
                {
                    ariles::write::Visitor::openFile(config_ofs_, file_name);
                    output_stream_ = &config_ofs_;
                    document_.SetObject();
                }


                explicit Writer(std::ostream &output_stream)
                {
                    output_stream_ = &output_stream;
                    document_.SetObject();
                }
            };
        }  // namespace impl
    }      // namespace ns_rapidjson
}  // namespace ariles


namespace ariles
{
    namespace ns_rapidjson
    {
        Writer::Writer(const std::string &file_name, const Flags &flags) : Base(flags)
        {
            impl_ = ImplPtr(new Impl(file_name));
        }


        Writer::Writer(std::ostream &output_stream, const Flags &flags) : Base(flags)
        {
            impl_ = ImplPtr(new Impl(output_stream));
        }



        void Writer::flush()
        {
            ::rapidjson::StringBuffer buffer;
            ::rapidjson::PrettyWriter< ::rapidjson::StringBuffer> writer(buffer);
            impl_->document_.Accept(writer);
            *impl_->output_stream_ << buffer.GetString() << std::endl;
            impl_->output_stream_->flush();
        }



        void Writer::descend(const std::string &map_name)
        {
            ::rapidjson::Value key(map_name.c_str(), impl_->document_.GetAllocator());
            ::rapidjson::Value value;
            impl_->getRawNode().AddMember(key, value, impl_->document_.GetAllocator());

            // hack, we assume that the last added
            // child is the last in the list
            const ::rapidjson::Value::MemberIterator child = --(impl_->getRawNode().MemberEnd());
            impl_->node_stack_.push_back(impl::Writer::NodeWrapper(&(child->value)));
        }

        void Writer::ascend()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startMap(const std::size_t /*num_entries*/)
        {
            impl_->getRawNode().SetObject();
            // not provided in older versions
            // impl_->getRawNode().MemberReserve(num_entries, impl_->document_.GetAllocator());
        }



        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            impl_->getRawNode().SetArray();
            impl_->getRawNode().Reserve(size, impl_->document_.GetAllocator());
            for (std::size_t i = 0; i < size; ++i)
            {
                ::rapidjson::Value value;
                impl_->getRawNode().PushBack(value, impl_->document_.GetAllocator());
            }
            impl_->node_stack_.push_back(impl::Writer::NodeWrapper(0, size));
        }

        void Writer::shiftArray()
        {
            ARILES_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ARILES_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
            ++impl_->node_stack_.back().index_;
        }

        void Writer::endArray()
        {
            impl_->node_stack_.pop_back();
        }


        /**
         * @brief Write a configuration entry
         *
         * @param[in] element data
         */
        void Writer::writeElement(const std::string &element)
        {
            impl_->getRawNode().SetString(element.c_str(), impl_->document_.GetAllocator());
        }

        void Writer::writeElement(const bool &element)
        {
            impl_->getRawNode().SetBool(element);
        }


        void Writer::writeElement(const float &element)
        {
            if (true == flags_.isSet(Flags::DISABLE_STRING_FLOATS))
            {
                impl_->getRawNode().SetDouble(element);  // old API compatibility
                // impl_->getRawNode().SetFloat(element);
            }
            else
            {
                impl_->getRawNode().SetString(
                        boost::lexical_cast<std::string>(element).c_str(), impl_->document_.GetAllocator());
            }
        }


        void Writer::writeElement(const double &element)
        {
            if (true == flags_.isSet(Flags::DISABLE_STRING_FLOATS))
            {
                impl_->getRawNode().SetDouble(element);
            }
            else
            {
                impl_->getRawNode().SetString(
                        boost::lexical_cast<std::string>(element).c_str(), impl_->document_.GetAllocator());
            }
        }



#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Writer::writeElement(const type &element)                                                                     \
    {                                                                                                                  \
        impl_->getRawNode().SetInt64(element);                                                                         \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES_BASIC_TYPE


#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Writer::writeElement(const type &element)                                                                     \
    {                                                                                                                  \
        impl_->getRawNode().SetUint64(element);                                                                        \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES_BASIC_TYPE
    }  // namespace ns_rapidjson
}  // namespace ariles
