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
        namespace ros
        {
            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer :
                public ariles::bridge::ros::Base<ariles::WriterBase>
            {
                public:
                    explicit Writer(const ::ros::NodeHandle &nh)
                    {
                        nh_ = nh;
                    }


                    void initRoot()
                    {
                        root_name_ = "";
                        root_value_.clear();
                    }


                    void flush()
                    {
                        if (XmlRpc::XmlRpcValue::TypeInvalid == root_value_.getType())
                        {
                            root_value_ = "";
                        }
                        nh_.setParam(root_name_, root_value_);
                        root_name_.clear();
                    }



                    void descend(const std::string &map_name)
                    {
                        if (0 == node_stack_.size())
                        {
                            root_name_ = map_name;
                            node_stack_.push_back(&root_value_);
                        }
                        else
                        {
                            node_stack_.push_back(   NodeWrapper(  &( getRawNode()[map_name] )  )   );
                        }
                    }


                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    void startArray(const std::size_t size, const bool /*compact*/ = false)
                    {
                        getRawNode().setSize(size);
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



                    void writeElement(const bool & element)
                    {
                        getRawNode() = element;
                    }


                    void writeElement(const std::string & element)
                    {
                        getRawNode() = element;
                    }


                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type & element) \
                        { \
                            getRawNode() = element; \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_REAL_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE



                    #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element) \
                            { \
                                ARILES_ASSERT(element <= std::numeric_limits<int>::max() \
                                              && element >= std::numeric_limits<int>::min(), \
                                              "Value is out of range."); \
                                getRawNode() = static_cast<int>(element); \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element) \
                            { \
                                ARILES_ASSERT(element <= std::numeric_limits<int>::max(), \
                                              "Value is too large."); \
                                getRawNode() = static_cast<int>(element); \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
