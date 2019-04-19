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

#include <iostream>

namespace ariles
{
    namespace bridge
    {
        namespace msgpack
        {
            namespace compact
            {
                /**
                 * @brief Configuration reader class
                 */
                class ARILES_VISIBILITY_ATTRIBUTE Reader :
                    public ariles::bridge::msgpack::Base<ariles::ReaderBase>
                {
                    protected:
                        typedef ariles::Node< const ::msgpack::object * > NodeWrapper;


                    protected:
                        std::string     buffer_;

                        ::msgpack::object_handle    handle_;

                        /// Stack of nodes.
                        std::vector<NodeWrapper>    node_stack_;


                    protected:
                        /**
                         * @brief open configuration file
                         *
                         * @param[in] input_stream
                         */
                        void initialize(std::istream & input_stream)
                        {
                            std::stringstream   str_stream;
                            str_stream << input_stream.rdbuf();
                            buffer_ = str_stream.str();

                            try
                            {
                                unpack(handle_, buffer_.data(), buffer_.size(), 0);
                                node_stack_.push_back( NodeWrapper( &handle_.get() ) );
                            }
                            catch(const std::exception &e)
                            {
                                ARILES_THROW(std::string("Failed to parse the configuration file: ") + e.what());
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



                        std::size_t getMapSize(const bool /*expect_empty*/)
                        {
                            return(getRawNode().via.array.size);
                        }

                        std::size_t startMapImpl(const std::size_t size)
                        {
                            node_stack_.push_back(NodeWrapper(0, size));
                            return (size);
                        }


                    public:
                        /**
                         * @brief Constructor
                         *
                         * @param[in] file_name
                         */
                        explicit Reader(const std::string& file_name)
                        {
                            std::ifstream config_ifs;
                            ReaderBase::openFile(config_ifs, file_name);
                            initialize(config_ifs);
                        }


                        /**
                         * @brief Constructor
                         *
                         * @param[in] input_stream
                         */
                        explicit Reader(std::istream & input_stream)
                        {
                            initialize(input_stream);
                        }


                        void endMap()
                        {
                            ARILES_ASSERT(
                                    true == node_stack_.back().isAllParsed(),
                                    "Some entries were not parsed, which is not allowed by this bridge.");
                            node_stack_.pop_back();
                        }


                        void ascend()
                        {
                            if(true == node_stack_.back().isArray())
                            {
                                shiftArray();
                            }
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
                            ARILES_ASSERT(true == node_stack_.back().isArray(),
                                          "Internal error: expected array.");
                            ARILES_ASSERT(node_stack_.back().index_ < node_stack_.back().size_,
                                          "Internal error: array has more elements than expected.");
                            ++node_stack_.back().index_;
                        }


                        #define ARILES_BASIC_TYPE(type) \
                            void readElement(type &element) \
                            { \
                                getRawNode() >> element; \
                            }

                        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                        #undef ARILES_BASIC_TYPE
                };
            }
        }
    }
}
