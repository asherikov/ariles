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
        typedef serialization::Node<const ::msgpack::object *> NodeWrapper;
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
                        unpack(handle_, buffer_.data(), buffer_.size(), 0);
                        node_stack_.push_back(NodeWrapper(&handle_.get()));
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
                    else
                    {
                        return (*node_stack_[depth].node_);
                    }
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
            impl_ = ImplPtr(new Impl());
            impl_->initialize(config_ifs);
        }


        Reader::Reader(std::istream &input_stream)
        {
            impl_ = ImplPtr(new Impl());
            impl_->initialize(input_stream);
        }


        std::size_t Reader::getMapSize(const bool /*expect_empty*/)
        {
            return (impl_->getRawNode().via.array.size);
        }

        std::size_t Reader::startMapImpl(const std::size_t size)
        {
            impl_->node_stack_.push_back(NodeWrapper(0, size));
            return (size);
        }


        void Reader::endMap()
        {
            ARILES2_ASSERT(
                    true == impl_->node_stack_.back().isAllParsed(),
                    "Some entries were not parsed, which is not allowed by this visitor.");
            impl_->node_stack_.pop_back();
        }


        void Reader::ascend()
        {
            if (true == impl_->node_stack_.back().isArray())
            {
                shiftArray();
            }
        }


        std::size_t Reader::startArray()
        {
            std::size_t size = impl_->getRawNode().via.array.size;
            impl_->node_stack_.push_back(NodeWrapper(0, size));

            return (size);
        }


        void Reader::endArray()
        {
            impl_->node_stack_.pop_back();
        }


        void Reader::shiftArray()
        {
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
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
