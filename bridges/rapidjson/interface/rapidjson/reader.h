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
             * @brief Configuration reader class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ReaderBase, public ariles::SloppyMapReaderBase
            {
                protected:
                    typedef ariles::Node< const ::rapidjson::Value > NodeWrapper;


                protected:
                    /// instance of the parser
                    ::rapidjson::Document document_;

                    /// Stack of nodes.
                    std::vector<NodeWrapper>    node_stack_;


                protected:
                    void initialize(std::istream & input_stream)
                    {
                        ::rapidjson::IStreamWrapper isw(input_stream);
                        document_.ParseStream(isw);
                        ARILES_ASSERT(false == document_.HasParseError(), "Parsing failed");
                    }


                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
                    const ::rapidjson::Value & getRawNode(const std::size_t depth)
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


                    const ::rapidjson::Value & getRawNode()
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
                        const ::rapidjson::Value::ConstMemberIterator child =
                            getRawNode().FindMember(child_name.c_str());

                        if (getRawNode().MemberEnd() == child)
                        {
                            return(false);
                        }
                        else
                        {
                            node_stack_.push_back(NodeWrapper(&(child->value)));
                            return(true);
                        }
                    }


                    template<int t_size_limit_type>
                    std::size_t startMap(
                            const std::size_t & min = 0,
                            const std::size_t & max = 0)
                    {
                        return (checkSize<RelaxedSizeLimitType<t_size_limit_type>::value>(
                                    getRawNode().MemberCount(), 
                                    min, 
                                    max));
                    }

                    void endMap()
                    {
                    }


                    /**
                     * @brief Ascend from the current entry to its parent.
                     */
                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    bool getEntryNames(std::vector<std::string> &child_names)
                    {
                        const ::rapidjson::Value & selected_node = getRawNode();

                        if(false == selected_node.IsObject())
                        {
                            return (false);
                        }
                        else
                        {
                            child_names.resize(selected_node.MemberCount());

                            std::size_t i = 0;
                            for(::rapidjson::Value::ConstMemberIterator it = selected_node.MemberBegin();
                                it != selected_node.MemberEnd();
                                ++it, ++i)
                            {
                                child_names[i] = it->name.GetString();
                            }
                            return (true);
                        }
                    }


                    std::size_t startArray()
                    {
                        std::size_t size = getRawNode().Size();
                        node_stack_.push_back(NodeWrapper(0, size));

                        return(size);
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


                    template<class t_ElementType>
                        void readElement(t_ElementType &element)
                    {
                        element = getRawNode().Get<t_ElementType>();
                    }


                    void readElement(std::string &element)
                    {
                        element = getRawNode().GetString();
                    }
            };


#ifdef ARILES_BRIDGE_jsonnet
            namespace jsonnet
            {
                class Reader : public rapidjson::Reader
                {
                    public:
                        explicit Reader(const std::string& file_name)
                        {
                            struct JsonnetVm* vm = static_cast<struct JsonnetVm*>(::jsonnet_make());
                            ARILES_ASSERT(NULL != vm, "Could not initialize jsonnet preprocessor.");

                            int error = 0;
                            const char* jsonnet_output = ::jsonnet_evaluate_file(vm, file_name.c_str(), &error);
                            ARILES_ASSERT(0 == error, jsonnet_output);

                            document_.Parse(jsonnet_output);

                            ::jsonnet_destroy(vm);
                        }

                        explicit Reader(std::istream & input_stream)
                        {
                            std::string input_string;
                            char buffer[4096];
                            while (input_stream.read(buffer, sizeof(buffer)))
                            {
                                input_string.append(buffer, sizeof(buffer));
                            }
                            input_string.append(buffer, input_stream.gcount());


                            struct JsonnetVm* vm = static_cast<struct JsonnetVm*>(::jsonnet_make());
                            ARILES_ASSERT(NULL != vm, "Could not initialize jsonnet preprocessor.");


                            int error = 0;
                            const char* jsonnet_output =
                                ::jsonnet_evaluate_snippet(vm, "<input steam>", input_string.c_str(), &error);
                            ARILES_ASSERT(0 == error, jsonnet_output);


                            document_.Parse(jsonnet_output);

                            ::jsonnet_destroy(vm);
                        }


                        ~Reader()
                        {
                        }
                };
            }
#endif
        }
    }
}
