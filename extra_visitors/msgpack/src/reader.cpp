/**
    @file
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <sstream>
#include <msgpack.hpp>

#include <ariles2/visitors/msgpack.h>

namespace ariles2
{
    namespace ns_msgpack
    {
        using NodeWrapper = serialization::Node<const ::msgpack::object *>;
    }
}  // namespace ariles2


namespace ariles2
{
    namespace ns_msgpack
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader
            {
            public:
                std::string buffer_;

                std::vector<std::shared_ptr<::msgpack::object_handle>> handles_;

                /// Stack of nodes.
                std::vector<NodeWrapper> node_stack_;

                std::size_t nameless_counter_;


            public:
                /**
                 * @brief open configuration file
                 *
                 * @param[in] input_stream
                 */
                void initialize(std::istream &input_stream)
                {
                    ARILES2_TRACE_FUNCTION;
                    std::stringstream str_stream;
                    str_stream << input_stream.rdbuf();
                    buffer_ = str_stream.str();


                    handles_.clear();
                    try
                    {
                        std::size_t buffer_offset = 0;

                        while (buffer_offset != buffer_.size())
                        {
                            handles_.push_back(std::make_shared<::msgpack::object_handle>());

                            unpack(*handles_.back(), buffer_.data(), buffer_.size(), buffer_offset);
                        }
                    }
                    catch (const std::exception &e)
                    {
                        ARILES2_THROW(std::string("Failed to parse the configuration file: ") + e.what());
                    }

                    nameless_counter_ = 0;
                }


                /**
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 */
                const ::msgpack::object &getRawNode(const std::size_t depth)
                {
                    ARILES2_TRACE_FUNCTION;
                    if (node_stack_[depth].isArray())
                    {
                        return (getRawNode(depth - 1).via.array.ptr[node_stack_[depth].index_]);
                    }
                    return (*node_stack_[depth].node_);
                }


                const ::msgpack::object &getRawNode()
                {
                    ARILES2_TRACE_FUNCTION;
                    return (getRawNode(node_stack_.size() - 1));
                }
            };
        }  // namespace impl
    }      // namespace ns_msgpack
}  // namespace ariles2


namespace ariles2
{
    namespace ns_msgpack
    {
        Reader::Reader(const std::string &file_name)
        {
            std::ifstream config_ifs;
            read::Visitor::openFile(config_ifs, file_name);
            makeImplPtr();
            impl_->initialize(config_ifs);
        }


        Reader::Reader(std::istream &input_stream)
        {
            makeImplPtr();
            impl_->initialize(input_stream);
        }


        void Reader::startMap(const SizeLimitEnforcementType limit_type, const std::size_t min, const std::size_t max)
        {
            ARILES2_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().via.map.size, min, max);
        }



        bool Reader::startMapEntry(const std::string &child_name)
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_TRACE_VALUE(child_name);
            if (impl_->node_stack_.empty())
            {
                for (std::size_t i = 0; i < impl_->handles_.size(); ++i)
                {
                    if (::msgpack::type::MAP == impl_->handles_[i]->get().type)
                    {
                        if (child_name == impl_->handles_[i]->get().via.map.ptr[0].key.as<std::string>())
                        {
                            if (::msgpack::type::MAP == impl_->handles_[i]->get().via.map.ptr[0].val.type)
                            {
                                impl_->node_stack_.emplace_back(&(impl_->handles_[i]->get().via.map.ptr[0].val));
                                return (true);
                            }
                        }
                    }
                }
            }
            else
            {
                if (::msgpack::type::MAP == impl_->getRawNode().type)
                {
                    for (std::size_t i = 0; i < impl_->getRawNode().via.map.size; ++i)
                    {
                        if (child_name == impl_->getRawNode().via.map.ptr[i].key.as<std::string>())
                        {
                            impl_->node_stack_.emplace_back(&(impl_->getRawNode().via.map.ptr[i].val));
                            return (true);
                        }
                    }
                }
            }

            return (false);
        }


        void Reader::endMapEntry()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            ARILES2_TRACE_FUNCTION;
            const std::size_t size = impl_->getRawNode().via.array.size;
            impl_->node_stack_.emplace_back(0, size);

            return (size);
        }


        void Reader::endArray()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


        void Reader::startArrayElement()
        {
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: namevalue.has more elements than expected.");
        }


        void Reader::endArrayElement()
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ++impl_->node_stack_.back().index_;
        }


        bool Reader::startRoot(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            if (name.empty())
            {
                ARILES2_ASSERT(
                        0 == impl_->nameless_counter_,
                        "Multiple nameless root entries are not supported, specify root names explicitly.");
                ++impl_->nameless_counter_;
                return (startMapEntry("ariles"));
            }
            return (startMapEntry(name));
        }

        void Reader::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            endMapEntry();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        impl_->getRawNode() >> element;                                                                                \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_msgpack
}  // namespace ariles2
