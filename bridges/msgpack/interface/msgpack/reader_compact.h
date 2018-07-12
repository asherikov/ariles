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
                class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ReaderBase
                {
                    protected:
                        typedef ariles::Node< const ::msgpack::object > NodeWrapper;


                    protected:
                        std::string     buffer_;

                        ::msgpack::object_handle    handle_;

                        /// Stack of nodes.
                        std::vector<NodeWrapper>    node_stack_;


                    protected:
                        /**
                         * @brief open configuration file
                         *
                         * @param[in] file_name
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
                            std::ifstream config_ifs;
                            ReaderBase::openFile(config_ifs, file_name);
                            initialize(config_ifs);
                        }


                        /**
                         * @brief Constructor
                         *
                         * @param[in] file_name
                         */
                        explicit Reader(std::istream & input_stream)
                        {
                            initialize(input_stream);
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
                        bool descend(const std::string & /*child_name*/)
                        {
                            return (true);
                        }

                        std::size_t startMap()
                        {
                            std::size_t size = getRawNode().via.array.size;
                            node_stack_.push_back(NodeWrapper(0, size));
                            return (size);
                        }

                        void endMap()
                        {
                            node_stack_.pop_back();
                        }

                        /**
                         * @brief Ascend from the current entry to its parent.
                         */
                        void ascend()
                        {
                            shiftArray();
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
    }
}
