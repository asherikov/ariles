/**
    @file
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    namespace msgpack
    {
        /**
         * @brief Configuration reader class
         */
        class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ReaderBase
        {
            protected:
                typedef ariles::Node< ::msgpack::object > NodeWrapper;


            protected:
                std::string     buffer_;

                std::vector< boost::shared_ptr< ::msgpack::object_handle >  >           handles_;

                /// Stack of nodes.
                std::vector<NodeWrapper>    node_stack_;


            protected:
                /**
                 * @brief open configuration file
                 *
                 * @param[in] file_name
                 */
                void openFile(const std::string& file_name)
                {
                    std::ifstream config_ifs_;

                    ReaderBase::openFile(config_ifs_, file_name);


                    std::stringstream   str_stream;
                    str_stream << config_ifs_.rdbuf();
                    buffer_ = str_stream.str();


                    handles_.clear();
                    try
                    {
                        std::size_t     buffer_offset = 0;

                        while (buffer_offset != buffer_.size())
                        {
                            handles_.push_back(boost::shared_ptr< ::msgpack::object_handle >(new ::msgpack::object_handle));

                            unpack(*handles_[handles_.size()-1], buffer_.data(), buffer_.size(), buffer_offset);
                        }
                    }
                    catch(const std::exception &e)
                    {
                        ARILES_THROW_MSG(std::string("Failed to parse the configuration file: ") + e.what());
                    }
                }


                /**
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 */
                const ::msgpack::object & getRawNode(const std::size_t depth)
                {
                    if (node_stack_[depth].isArray())
                    {
                        return(getRawNode(depth-1).via.array.ptr[node_stack_[depth].index_]);
                    }
                    else
                    {
                        return(*node_stack_[depth].node_);
                    }
                }


                const ::msgpack::object & getRawNode()
                {
                    return(getRawNode(node_stack_.size()-1));
                }


            public:
                /**
                 * @brief Constructor
                 *
                 * @param[in] file_name
                 */
                explicit Reader(const std::string& file_name)
                {
                    openFile(file_name);
                }


                /**
                 * @brief Default constructor
                 */
                Reader()
                {
                }


                /**
                 * @brief Descend to the entry with the given name
                 *
                 * @param[in] child_name child node name
                 *
                 * @return true if successful.
                 */
                bool descend(const std::string & child_name)
                {
                    if (node_stack_.size() == 0)
                    {
                        for (std::size_t i = 0; i < handles_.size(); ++i)
                        {
                            if (::msgpack::type::MAP == handles_[i]->get().type)
                            {
                                if (child_name == handles_[i]->get().via.map.ptr[0].key.as<std::string>())
                                {
                                    if (::msgpack::type::MAP == handles_[i]->get().via.map.ptr[0].val.type)
                                    {
                                        node_stack_.push_back( NodeWrapper( &(handles_[i]->get().via.map.ptr[0].val) ) );
                                        return(true);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        if (::msgpack::type::MAP == getRawNode().type)
                        {
                            for (std::size_t i = 0; i < getRawNode().via.map.size; ++i)
                            {
                                if (child_name == getRawNode().via.map.ptr[i].key.as<std::string>())
                                {
                                    node_stack_.push_back( NodeWrapper( &(getRawNode().via.map.ptr[i].val) ) );
                                    return(true);
                                }
                            }
                        }
                    }

                    return (false);
                }


                /**
                 * @brief Ascend from the current entry to its parent.
                 */
                void ascend()
                {
                    node_stack_.pop_back();
                }


                std::size_t startArray()
                {
                    std::size_t size = getRawNode().via.array.size;
                    node_stack_.push_back(NodeWrapper(0, size));

                    return(size);
                }


                void endArray()
                {
                    node_stack_.pop_back();
                }


                void shiftArray()
                {
                    if (node_stack_.back().isArray())
                    {
                        ++node_stack_.back().index_;
                    }
                }


                template<class t_ElementType>
                    void readElement(t_ElementType &element)
                {
                    getRawNode() >> element;
                }
        };
    }
}
