/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    namespace bridge
    {
        namespace pugixml
        {
            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer : public ariles::WriterBase
            {
                protected:
                    typedef ariles::Node< pugi::xml_node > NodeWrapper;


                protected:
                    std::vector<NodeWrapper>    node_stack_;

                    /// output file stream
                    std::ofstream   config_ofs_;

                    /// output stream
                    std::ostream    *output_stream_;

                    pugi::xml_document document_;


                protected:
                    pugi::xml_node & getRawNode()
                    {
                        return(node_stack_.back().node_);
                    }


                public:
                    explicit Writer(const std::string& file_name)
                    {
                        WriterBase::openFile(config_ofs_, file_name);
                        output_stream_ = &config_ofs_;
                        node_stack_.push_back(document_);
                    }


                    explicit Writer(std::ostream& output_stream)
                    {
                        output_stream_ = &output_stream;
                        node_stack_.push_back(document_);
                    }


                    ~Writer()
                    {
                    }


                    const BridgeParameters &getBridgeParameters() const
                    {
                        static BridgeParameters parameters(true);
                        return (parameters);
                    }



                    /**
                     * @brief Flush the configuration to the file
                     */
                    void flush()
                    {
                        document_.save(*output_stream_, "    ", pugi::format_indent);
                        output_stream_->flush();
                    }


                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] map_name name of the map
                     */
                    void descend(const std::string &map_name)
                    {
                        node_stack_.push_back( getRawNode().append_child(map_name.c_str()) );
                    }

                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] num_entries number of child entries
                     */
                    void startMap(const std::size_t /*num_entries*/)
                    {
                    }


                    void startArray(const std::size_t size, const bool /*compact*/ = false)
                    {
                        node_stack_.push_back(NodeWrapper(getRawNode(), 0, size));
                        if (size > 0)
                        {
                            node_stack_.push_back( getRawNode().append_child("item") );
                        }
                    }

                    void shiftArray()
                    {
                        ARILES_ASSERT(true == node_stack_.back().isArray(),
                                      "Internal error: expected array.");
                        ARILES_ASSERT(node_stack_.back().index_ < node_stack_.back().size_,
                                      "Internal error: array has more elements than expected.");
                        node_stack_.pop_back();
                        node_stack_.push_back( getRawNode().append_child("item") );
                        ++node_stack_.back().index_;
                    }

                    void endArray()
                    {
                        node_stack_.pop_back();
                    }



                    /**
                     * @brief Write a configuration entry
                     *
                     * @tparam t_EntryType type of the entry
                     *
                     * @param[in] entry      data
                     */
                    void writeElement(const std::string & element)
                    {
                        getRawNode().text() = element.c_str();
                    }

                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type & element) \
                        { \
                            getRawNode().text() = element; \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_NUMERIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
