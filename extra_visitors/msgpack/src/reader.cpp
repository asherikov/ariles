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

#include <ariles/visitors/msgpack.h>

namespace ariles
{
    namespace ns_msgpack
    {
        typedef ariles::Node<const ::msgpack::object *> NodeWrapper;
    }
}  // namespace ariles


namespace ariles
{
    namespace ns_msgpack
    {
        namespace impl
        {
            class ARILES_LIB_LOCAL Reader
            {
            public:
                std::string buffer_;

                std::vector<ARILES_SHARED_PTR< ::msgpack::object_handle> > handles_;

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
                    ARILES_TRACE_FUNCTION;
                    std::stringstream str_stream;
                    str_stream << input_stream.rdbuf();
                    buffer_ = str_stream.str();


                    handles_.clear();
                    try
                    {
                        std::size_t buffer_offset = 0;

                        while (buffer_offset != buffer_.size())
                        {
                            handles_.push_back(ARILES_SHARED_PTR< ::msgpack::object_handle>(
                                    new ::msgpack::object_handle));

                            unpack(*handles_[handles_.size() - 1],
                                   buffer_.data(),
                                   buffer_.size(),
                                   buffer_offset);
                        }
                    }
                    catch (const std::exception &e)
                    {
                        ARILES_THROW(
                                std::string("Failed to parse the configuration file: ") + e.what());
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
                    ARILES_TRACE_FUNCTION;
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
                    ARILES_TRACE_FUNCTION;
                    return (getRawNode(node_stack_.size() - 1));
                }
            };
        }  // namespace impl
    }      // namespace ns_msgpack
}  // namespace ariles


namespace ariles
{
    namespace ns_msgpack
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
            ARILES_TRACE_FUNCTION;
            return (impl_->getRawNode().via.map.size);
        }



        bool Reader::descend(const std::string &child_name)
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(child_name);
            if (impl_->node_stack_.size() == 0)
            {
                for (std::size_t i = 0; i < impl_->handles_.size(); ++i)
                {
                    if (::msgpack::type::MAP == impl_->handles_[i]->get().type)
                    {
                        if (child_name
                            == impl_->handles_[i]->get().via.map.ptr[0].key.as<std::string>())
                        {
                            if (::msgpack::type::MAP
                                == impl_->handles_[i]->get().via.map.ptr[0].val.type)
                            {
                                impl_->node_stack_.push_back(NodeWrapper(
                                        &(impl_->handles_[i]->get().via.map.ptr[0].val)));
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
                            impl_->node_stack_.push_back(
                                    NodeWrapper(&(impl_->getRawNode().via.map.ptr[i].val)));
                            return (true);
                        }
                    }
                }
            }

            return (false);
        }


        void Reader::ascend()
        {
            ARILES_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            ARILES_TRACE_FUNCTION;
            std::size_t size = impl_->getRawNode().via.array.size;
            impl_->node_stack_.push_back(NodeWrapper(0, size));

            return (size);
        }


        void Reader::endArray()
        {
            ARILES_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


        void Reader::shiftArray()
        {
            ARILES_TRACE_FUNCTION;
            ARILES_ASSERT(
                    true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ARILES_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
            ++impl_->node_stack_.back().index_;
        }


        bool Reader::startRoot(const std::string &name)
        {
            ARILES_TRACE_FUNCTION;
            if (true == name.empty())
            {
                ARILES_ASSERT(
                        0 == impl_->nameless_counter_,
                        "Multiple nameless root entries are not supported, specify root names explicitly.");
                ++impl_->nameless_counter_;
                return (descend("ariles"));
            }
            else
            {
                return (descend(name));
            }
        }

        void Reader::endRoot(const std::string & /*name*/)
        {
            ARILES_TRACE_FUNCTION;
            ascend();
        }


#define ARILES_BASIC_TYPE(type)                                                                    \
    void Reader::readElement(type &element)                                                        \
    {                                                                                              \
        ARILES_TRACE_FUNCTION;                                                                     \
        impl_->getRawNode() >> element;                                                            \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

#undef ARILES_BASIC_TYPE
    }  // namespace ns_msgpack
}  // namespace ariles
