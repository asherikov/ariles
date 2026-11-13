/**
    @file
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iostream>
#include <sstream>

#include <ariles2/visitors/msgpack.h>
#include "reader_common.h"


namespace ariles2
{
    namespace ns_msgpack_compact
    {
        using NodeWrapper = serialization::Node<const msgpack_object *>;

        using ns_msgpack_c::readValue;


        namespace impl
        {
            class Reader : public serialization::NodeStackBase<NodeWrapper>, public read::FileVisitorImplementation
            {
            public:
                std::string buffer_;

                msgpack_unpacked handle_ = {};


            public:
                Reader(const Reader &) = delete;
                void operator=(const Reader &) = delete;

                ~Reader()
                {
                    msgpack_unpacked_destroy(&handle_);
                }


                template <class... t_Args>
                explicit Reader(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                    initialize();
                }


                /**
                 * @brief open configuration file
                 */
                void initialize()
                {
                    std::stringstream str_stream;
                    str_stream << input_stream_->rdbuf();
                    buffer_ = str_stream.str();

                    if (MSGPACK_UNPACK_SUCCESS
                        != msgpack_unpack_next(&handle_, buffer_.data(), buffer_.size(), nullptr))
                    {
                        CPPUT_THROW("Failed to parse the configuration file.");
                    }
                    emplace(&handle_.data);
                }


                /**
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 */
                const msgpack_object &getRawNode(const std::size_t depth)
                {
                    if (node_stack_[depth].isArray())
                    {
                        return (getRawNode(depth - 1).via.array.ptr[node_stack_[depth].index_]);
                    }
                    return (*node_stack_[depth].node_);
                }


                const msgpack_object &getRawNode()
                {
                    return (getRawNode(node_stack_.size() - 1));
                }
            };
        }  // namespace impl
    }  // namespace ns_msgpack_compact
}  // namespace ariles2



namespace ariles2
{
    namespace ns_msgpack_compact
    {
        Reader::Reader(const std::string &file_name)
        {
            makeImplPtr(file_name);
        }


        Reader::Reader(std::istream &input_stream)
        {
            makeImplPtr(input_stream);
        }


        void Reader::startMap(const SizeLimitEnforcementType limit_type, const std::size_t min, const std::size_t max)
        {
            const std::size_t size = impl_->getRawNode().via.array.size;
            checkSize(limit_type, size, min, max);
            impl_->emplace(0, size);
        }

        bool Reader::startMapEntry(const std::string &)
        {
            if (impl_->back().isArray())
            {
                startArrayElement();
            }
            return (true);
        }

        void Reader::endMapEntry()
        {
            if (impl_->back().isArray())
            {
                endArrayElement();
            }
        }

        void Reader::endMap()
        {
            CPPUT_ASSERT(
                    impl_->back().isCompleted(), "Some entries were not parsed, which is not allowed by this visitor.");
            impl_->pop();
        }


        std::size_t Reader::startArray()
        {
            const std::size_t size = impl_->getRawNode().via.array.size;
            impl_->emplace(0, size);

            return (size);
        }


        void Reader::endArray()
        {
            impl_->pop();
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


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        readValue(impl_->getRawNode(), element);                                                                       \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_msgpack_compact
}  // namespace ariles2
