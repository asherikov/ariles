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

#include <ariles2/visitors/msgpack.h>
#include "reader_common.h"


namespace ariles2
{
    namespace ns_msgpack
    {
        using NodeWrapper = serialization::Node<const msgpack_object *>;

        using ns_msgpack_c::isEqual;
        using ns_msgpack_c::readValue;


        namespace impl
        {
            class Reader : public serialization::NodeStackBase<NodeWrapper>, public read::FileVisitorImplementation
            {
            public:
                std::string buffer_;

                std::vector<msgpack_unpacked> handles_;

                std::size_t nameless_counter_;


            public:
                Reader(const Reader &) = delete;
                void operator=(const Reader &) = delete;

                ~Reader()
                {
                    clear();
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
                    CPPUT_TRACE_FUNCTION;
                    std::stringstream str_stream;
                    str_stream << input_stream_->rdbuf();
                    buffer_ = str_stream.str();

                    clear();

                    std::size_t buffer_offset = 0;

                    while (buffer_offset < buffer_.size())
                    {
                        msgpack_unpacked handle = {};
                        if (MSGPACK_UNPACK_SUCCESS
                            != msgpack_unpack_next(&handle, buffer_.data(), buffer_.size(), &buffer_offset))
                        {
                            msgpack_unpacked_destroy(&handle);
                            CPPUT_THROW("Failed to parse the configuration file.");
                        }
                        handles_.push_back(handle);
                    }

                    nameless_counter_ = 0;
                }


                void clear()
                {
                    for (msgpack_unpacked &handle : handles_)
                    {
                        msgpack_unpacked_destroy(&handle);
                    }
                    handles_.clear();
                }


                /**
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 */
                const msgpack_object &getRawNode(const std::size_t depth)
                {
                    CPPUT_TRACE_FUNCTION;
                    if (node_stack_[depth].isArray())
                    {
                        return (getRawNode(depth - 1).via.array.ptr[node_stack_[depth].index_]);
                    }
                    return (*node_stack_[depth].node_);
                }


                const msgpack_object &getRawNode()
                {
                    CPPUT_TRACE_FUNCTION;
                    return (getRawNode(node_stack_.size() - 1));
                }
            };
        }  // namespace impl
    }  // namespace ns_msgpack
}  // namespace ariles2


namespace ariles2
{
    namespace ns_msgpack
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
            CPPUT_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().via.map.size, min, max);
        }



        bool Reader::startMapEntry(const std::string &child_name)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_TRACE_VALUE(child_name);
            if (impl_->empty())
            {
                for (const msgpack_unpacked &handle : impl_->handles_)
                {
                    const msgpack_object &root = handle.data;
                    if (MSGPACK_OBJECT_MAP == root.type && root.via.map.size > 0
                        && isEqual(root.via.map.ptr[0].key, child_name)
                        && MSGPACK_OBJECT_MAP == root.via.map.ptr[0].val.type)
                    {
                        impl_->emplace(&(root.via.map.ptr[0].val));
                        return (true);
                    }
                }
            }
            else
            {
                if (MSGPACK_OBJECT_MAP == impl_->getRawNode().type)
                {
                    const msgpack_object_map &map = impl_->getRawNode().via.map;
                    for (std::size_t i = 0; i < map.size; ++i)
                    {
                        if (isEqual(map.ptr[i].key, child_name))
                        {
                            impl_->emplace(&(map.ptr[i].val));
                            return (true);
                        }
                    }
                }
            }

            return (false);
        }


        void Reader::endMapEntry()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
        }


        std::size_t Reader::startArray()
        {
            CPPUT_TRACE_FUNCTION;
            const std::size_t size = impl_->getRawNode().via.array.size;
            impl_->emplace(0, size);

            return (size);
        }


        void Reader::endArray()
        {
            CPPUT_TRACE_FUNCTION;
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
            CPPUT_TRACE_FUNCTION;
            impl_->shiftArray();
        }


        bool Reader::startRoot(const std::string &name)
        {
            CPPUT_TRACE_FUNCTION;
            if (name.empty())
            {
                CPPUT_ASSERT(
                        0 == impl_->nameless_counter_,
                        "Multiple nameless root entries are not supported, specify root names explicitly.");
                ++impl_->nameless_counter_;
                return (startMapEntry("ariles"));
            }
            return (startMapEntry(name));
        }

        void Reader::endRoot(const std::string & /*name*/)
        {
            CPPUT_TRACE_FUNCTION;
            endMapEntry();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        readValue(impl_->getRawNode(), element);                                                                       \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_msgpack
}  // namespace ariles2
