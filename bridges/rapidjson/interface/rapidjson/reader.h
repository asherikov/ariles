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
            class ARILES_VISIBILITY_ATTRIBUTE Reader :
                public ariles::bridge::rapidjson::Base<ariles::ReaderBase, const ::rapidjson::Value>
            {
                protected:
                    void initialize(std::istream & input_stream)
                    {
                        ::rapidjson::IStreamWrapper isw(input_stream);
                        document_.ParseStream(isw);
                        ARILES_ASSERT(false == document_.HasParseError(), "Parsing failed");
                    }


                    std::size_t getMapSize()
                    {
                        return (getRawNode().MemberCount());
                    }


                protected:
                    Reader()
                    {
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


                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    bool getMapEntryNames(std::vector<std::string> &child_names)
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
                        ARILES_ASSERT(true == node_stack_.back().isArray(),
                                      "Internal error: expected array.");
                        ARILES_ASSERT(node_stack_.back().index_ < node_stack_.back().size_,
                                      "Internal error: array has more elements than expected.");
                        ++node_stack_.back().index_;
                    }


                    void endArray()
                    {
                        node_stack_.pop_back();
                    }


                    void readElement(std::string &element)
                    {
                        element = getRawNode().GetString();
                    }

                    void readElement(bool &element)
                    {
                        element = getRawNode().Get<bool>();
                    }


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element) \
                        { \
                            element = getRawNode().Get<int>(); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element) \
                        { \
                            element = getRawNode().Get<unsigned int>(); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element) \
                        { \
                            element = getRawNode().Get<type>(); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_REAL_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
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
