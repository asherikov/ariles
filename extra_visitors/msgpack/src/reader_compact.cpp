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
#include <iostream>
#include <msgpack.hpp>

#include <ariles2/visitors/msgpack.h>


namespace ariles2
{
    namespace ns_msgpack_compact
    {
        using NodeWrapper = serialization::Node<const ::msgpack::object *>;
    }
}  // namespace ariles2


namespace ariles2
{
    namespace ns_msgpack_compact
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader
            {
            public:
                std::string buffer_;

                ::msgpack::object_handle handle_;

                /// Stack of nodes.
                std::vector<NodeWrapper> node_stack_;


            public:
                /**
                 * @brief open configuration file
                 *
                 * @param[in] input_stream
                 */
                void initialize(std::istream &input_stream)
                {
                    std::stringstream str_stream;
                    str_stream << input_stream.rdbuf();
                    buffer_ = str_stream.str();

                    try
                    {
                        unpack(handle_, buffer_.data(), buffer_.size(), NULL);
                        node_stack_.emplace_back(&handle_.get());
                    }
                    catch (const std::exception &e)
                    {
                        ARILES2_THROW(std::string("Failed to parse the configuration file: ") + e.what());
                    }
                }


                /**
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 */
                const ::msgpack::object &getRawNode(const std::size_t depth)
                {
                    if (node_stack_[depth].isArray())
                    {
                        return (getRawNode(depth - 1).via.array.ptr[node_stack_[depth].index_]);
                    }
                    return (*node_stack_[depth].node_);
                }


                const ::msgpack::object &getRawNode()
                {
                    return (getRawNode(node_stack_.size() - 1));
                }
            };
        }  // namespace impl
    }      // namespace ns_msgpack_compact
}  // namespace ariles2



namespace ariles2
{
    namespace ns_msgpack_compact
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
            const std::size_t size = impl_->getRawNode().via.array.size;
            checkSize(limit_type, size, min, max);
            impl_->node_stack_.emplace_back(0, size);
        }

        bool Reader::startMapEntry(const std::string &)
        {
            if (impl_->node_stack_.back().isArray())
            {
                startArrayElement();
            }
            return (true);
        }

        void Reader::endMapEntry()
        {
            if (impl_->node_stack_.back().isArray())
            {
                endArrayElement();
            }
        }

        void Reader::endMap()
        {
            ARILES2_ASSERT(
                    impl_->node_stack_.back().isAllParsed(),
                    "Some entries were not parsed, which is not allowed by this visitor.");
            impl_->node_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            std::size_t size = impl_->getRawNode().via.array.size;
            impl_->node_stack_.emplace_back(0, size);

            return (size);
        }


        void Reader::endArray()
        {
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
            ARILES2_ASSERT(impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ++impl_->node_stack_.back().index_;
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        impl_->getRawNode() >> element;                                                                                \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_msgpack_compact
}  // namespace ariles2
