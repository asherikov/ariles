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
             * @brief Configuration reader class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ReaderBase
            {
                protected:
                    typedef ariles::Node< pugi::xml_node > NodeWrapper;


                protected:
                    /// instance of the parser
                    pugi::xml_document document_;

                    /// Stack of nodes.
                    std::vector<NodeWrapper>    node_stack_;


                protected:
                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
                    const pugi::xml_node & getRawNode()
                    {
                        return(node_stack_.back().node_);
                    }



                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] file_name
                     */
                    explicit Reader(const std::string& file_name)
                    {
                        pugi::xml_parse_result result = document_.load_file(
                                file_name.c_str(),
                                pugi::parse_minimal);
                        ARILES_ASSERT(true == result, std::string("Parsing failed: ") + result.description());
                    }


                    /**
                     * @brief Constructor
                     *
                     * @param[in] input_stream
                     */
                    explicit Reader(std::istream & input_stream)
                    {
                        pugi::xml_parse_result result = document_.load(
                                input_stream,
                                pugi::parse_minimal);
                        ARILES_ASSERT(true == result, std::string("Parsing failed: ") + result.description());
                    }


                    /**
                     * @brief Default constructor
                     */
                    Reader()
                    {
                    }


                    const BridgeParameters &getBridgeParameters() const
                    {
                        static BridgeParameters parameters(true);
                        return (parameters);
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
                        const pugi::xml_node child = getRawNode().child(child_name.c_str());

                        if (child)
                        {
                            node_stack_.push_back(child);
                            return(true);
                        }
                        else
                        {
                            return(false);
                        }
                    }


                    /**
                     * @brief Ascend from the current entry to its parent.
                     */
                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    bool getMapEntryNames(std::vector<std::string> &child_names)
                    {
                        const pugi::xml_node node = getRawNode();
                        for (pugi::xml_node_iterator it = node.begin(); it != node.end(); ++it)
                        {
                            child_names.clear();
                            child_names.push_back(it->name());
                        }
                        return (true);
                    }


                    std::size_t startArray()
                    {
                        std::size_t size = 0;
                        const pugi::xml_node node = getRawNode();
                        for(pugi::xml_node child = node.child("item");
                            child;
                            child = child.next_sibling("item"), ++size);

                        node_stack_.push_back(NodeWrapper(node.child("item"), 0, size));

                        return(size);
                    }


                    void shiftArray()
                    {
                        ARILES_ASSERT(true == node_stack_.back().isArray(),
                                      "Internal error: expected array.");
                        ARILES_ASSERT(node_stack_.back().index_ < node_stack_.back().size_,
                                      "Internal error: array has more elements than expected.");
                        node_stack_.back().node_ = getRawNode().next_sibling("item");
                        ++node_stack_.back().index_;
                    }


                    void endArray()
                    {
                        node_stack_.pop_back();
                    }


                    void readElement(std::string &element)
                    {
                        element = getRawNode().text().as_string();
                    }

                    void readElement(bool &element)
                    {
                        ARILES_ASSERT(false == getRawNode().text().empty(),
                                    "Empty bool elements are not allowed.");
                        element = getRawNode().text().as_bool();
                    }


                    void readElement(float &element)
                    {
                        ARILES_ASSERT(false == getRawNode().text().empty(),
                                    "Empty bool elements are not allowed.");
                        element = getRawNode().text().as_float();
                    }


                    void readElement(double &element)
                    {
                        ARILES_ASSERT(false == getRawNode().text().empty(),
                                    "Empty bool elements are not allowed.");
                        element = getRawNode().text().as_double();
                    }


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element) \
                        { \
                            ARILES_ASSERT(false == getRawNode().text().empty(), \
                                        "Empty integer elements are not allowed."); \
                            long long tmp = getRawNode().text().as_llong(); \
                            ARILES_ASSERT(tmp <= std::numeric_limits<type>::max() \
                                        && tmp >= std::numeric_limits<type>::min(), \
                                        "Value is out of range."); \
                            element = static_cast<type>(tmp); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element) \
                        { \
                            ARILES_ASSERT(false == getRawNode().text().empty(), \
                                        "Empty integer elements are not allowed."); \
                            unsigned long long tmp = getRawNode().text().as_ullong(); \
                            ARILES_ASSERT(tmp <= std::numeric_limits<type>::max(), \
                                        "Value is out of range."); \
                            element = static_cast<type>(tmp); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
