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
        namespace rapidjson
        {
            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer : public ariles::WriterBase
            {
                protected:
                    typedef ariles::Node< ::rapidjson::Value > NodeWrapper;


                protected:
                    std::vector<NodeWrapper>    node_stack_;

                    /// output file stream
                    std::ofstream   config_ofs_;

                    /// output stream
                    std::ostream    *output_stream_;

                    ::rapidjson::Document document_;


                protected:
                    ::rapidjson::Value & getRawNode(const std::size_t depth)
                    {
                        if (node_stack_[depth].isArray())
                        {
                            return(getRawNode(depth-1)[node_stack_[depth].index_]);
                        }
                        else
                        {
                            return(*node_stack_[depth].node_);
                        }
                    }


                    ::rapidjson::Value & getRawNode()
                    {
                        if (true == node_stack_.empty())
                        {
                            return (document_);
                        }
                        else
                        {
                            return (getRawNode(node_stack_.size()-1));
                        }
                    }


                public:
                    explicit Writer(const std::string& file_name)
                    {
                        WriterBase::openFile(config_ofs_, file_name);
                        output_stream_ = &config_ofs_;
                        document_.SetObject();
                    }


                    explicit Writer(std::ostream& output_stream)
                    {
                        output_stream_ = &output_stream;
                        document_.SetObject();
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
                        ::rapidjson::StringBuffer buffer;
                        ::rapidjson::PrettyWriter< ::rapidjson::StringBuffer > writer(buffer);
                        document_.Accept(writer);
                        *output_stream_ << buffer.GetString() << std::endl;
                        output_stream_->flush();
                    }



                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] map_name name of the map
                     */
                    void descend(const std::string &map_name)
                    {
                        ::rapidjson::Value key(map_name.c_str(), document_.GetAllocator());
                        ::rapidjson::Value value;
                        getRawNode().AddMember(
                            key,
                            value,
                            document_.GetAllocator());

                        // hack, we assume that the last added
                        // child is the last in the list
                        const ::rapidjson::Value::MemberIterator child =
                            --(getRawNode().MemberEnd());
                        node_stack_.push_back(NodeWrapper(&(child->value)));
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
                        getRawNode().SetObject();
                        // not provided in older versions
                        //getRawNode().MemberReserve(num_entries, document_.GetAllocator());
                    }



                    void startArray(const std::size_t size, const bool /*compact*/ = false)
                    {
                        getRawNode().SetArray();
                        getRawNode().Reserve(size, document_.GetAllocator());
                        for (std::size_t i = 0; i < size; ++i)
                        {
                            ::rapidjson::Value value;
                            getRawNode().PushBack(value, document_.GetAllocator());
                        }
                        node_stack_.push_back(NodeWrapper(0, size));
                    }

                    void shiftArray()
                    {
                        if (node_stack_.back().isArray())
                        {
                            ++node_stack_.back().index_;
                        }
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
                        getRawNode().SetString(element.c_str(), document_.GetAllocator());
                    }


                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type & element) \
                        { \
                            getRawNode() = element; \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_NUMERIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };


#ifdef ARILES_BRIDGE_jsonnet
            namespace jsonnet
            {
                // Useless, added for API symmetry
                typedef rapidjson::Writer Writer;
            }
#endif
        }
    }
}
