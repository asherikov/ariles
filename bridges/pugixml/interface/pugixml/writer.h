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
            class ARILES_VISIBILITY_ATTRIBUTE Writer :
                public ariles::bridge::pugixml::Base<ariles::WriterBase>
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
                        node_stack_.push_back(document_);
                    }


                    explicit Writer(std::ostream& output_stream)
                    {
                        output_stream_ = &output_stream;
                        node_stack_.push_back(document_);
                    }



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
                        node_stack_.pop_back();
                        ARILES_ASSERT(true == node_stack_.back().isArray(),
                                      "Internal error: expected array.");
                        ARILES_ASSERT(node_stack_.back().index_ < node_stack_.back().size_,
                                      "Internal error: array has more elements than expected.");
                        ++node_stack_.back().index_;
                        if (node_stack_.back().index_ < node_stack_.back().size_)
                        {
                            node_stack_.push_back( getRawNode().append_child("item") );
                        }
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
                        getRawNode().text() = element.c_str();
                    }


                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type & element) \
                        { \
                            getRawNode().text() = (boost::lexical_cast<std::string>(element)).c_str(); \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_NUMERIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
