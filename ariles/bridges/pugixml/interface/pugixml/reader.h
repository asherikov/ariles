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
            class ARILES_VISIBILITY_ATTRIBUTE Reader :
                public ariles::bridge::pugixml::Base<ariles::ReaderBase>
            {
                protected:
                    std::size_t getMapSize(const bool /*expect_empty*/)
                    {
                        std::size_t size = 0;
                        for(pugi::xml_node_iterator it = getRawNode().begin();
                            it != getRawNode().end();
                            ++it, ++size);
                        for(pugi::xml_attribute attribute = getRawNode().first_attribute();
                            attribute;
                            attribute = attribute.next_attribute(), ++size);
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
                        pugi::xml_parse_result result = document_.load_file(
                                file_name.c_str(),
                                pugi::parse_minimal);
                        ARILES_ASSERT(true == result, std::string("Parsing failed: ") + result.description());
                        node_stack_.push_back(document_);
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
                        node_stack_.push_back(document_);
                    }


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
                            const pugi::xml_attribute attribute = getRawNode().attribute(child_name.c_str());
                            if (attribute)
                            {
                                pugi::xml_node new_child = getRawNode().append_child(child_name.c_str());
                                new_child.text() = attribute.value();
                                node_stack_.push_back(new_child);
                                return(true);
                            }
                            else
                            {
                                return(false);
                            }
                        }
                    }


                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    bool getMapEntryNames(std::vector<std::string> &child_names)
                    {
                        const pugi::xml_node node = getRawNode();
                        child_names.clear();
                        for (pugi::xml_node_iterator it = node.begin(); it != node.end(); ++it)
                        {
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

                        if (size > 0)
                        {
                            node_stack_.push_back(NodeWrapper(node.child("item"), 0, size));
                        }
                        else
                        {
                            // if there are no 'item' childs try to iterate
                            // over childs with the same name in the parent
                            // node
                            for (pugi::xml_node child = getRawNode(); child; child = child.next_sibling(child.name()), ++size);
                            node_stack_.push_back(NodeWrapper(getRawNode(), 0, size));
                        }

                        return(size);
                    }


                    void shiftArray()
                    {
                        ARILES_ASSERT(true == node_stack_.back().isArray(),
                                      "Internal error: expected array.");
                        ARILES_ASSERT(node_stack_.back().index_ < node_stack_.back().size_,
                                      "Internal error: array has more elements than expected.");
                        node_stack_.back().node_ = getRawNode().next_sibling(getRawNode().name());
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


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element) \
                        { \
                            ARILES_ASSERT(false == getRawNode().text().empty(), \
                                        "Empty integer elements are not allowed."); \
                            element = boost::lexical_cast<type>(getRawNode().text().as_string()); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_NUMERIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
