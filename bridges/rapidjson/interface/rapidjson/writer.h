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
            class ARILES_VISIBILITY_ATTRIBUTE Writer :
                public ariles::bridge::rapidjson::Base<ariles::WriterBase, ::rapidjson::Value>
            {
                protected:
                    /// output file stream
                    std::ofstream   config_ofs_;

                    /// output stream
                    std::ostream    *output_stream_;


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



                    void flush()
                    {
                        ::rapidjson::StringBuffer buffer;
                        ::rapidjson::PrettyWriter< ::rapidjson::StringBuffer > writer(buffer);
                        document_.Accept(writer);
                        *output_stream_ << buffer.GetString() << std::endl;
                        output_stream_->flush();
                    }



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



                    /**
                     * @brief Write a configuration entry
                     *
                     * @param[in] element data
                     */
                    void writeElement(const std::string & element)
                    {
                        getRawNode().SetString(element.c_str(), document_.GetAllocator());
                    }

                    void writeElement(const bool & element)
                    {
                        getRawNode().SetBool(element);
                    }


                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type &element) \
                        { \
                            getRawNode().SetString(boost::lexical_cast<std::string>(element).c_str(), document_.GetAllocator()); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_REAL_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type &element) \
                        { \
                            getRawNode().SetInt64(element); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type &element) \
                        { \
                            getRawNode().SetUint64(element); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };


#ifdef ARILES_BRIDGE_INCLUDED_jsonnet
            namespace jsonnet
            {
                // Useless, added for API symmetry
                typedef rapidjson::Writer Writer;
            }
#endif
        }
    }
}
