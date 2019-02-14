/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/lexical_cast.hpp>

namespace ariles
{
    namespace bridge
    {
        namespace ros
        {
            /**
             * @brief Configuration reader class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Reader :
                public ariles::bridge::ros::Base<ariles::ReaderBase>
            {
                protected:
                    std::size_t getMapSize(const bool expect_empty)
                    {
                        if (XmlRpc::XmlRpcValue::TypeStruct == getRawNode().getType())
                        {
                            return (getRawNode().size());
                        }
                        else
                        {
                            ARILES_PERSISTENT_ASSERT(true == expect_empty, "Expected struct.");
                            return (0);
                        }
                    }


                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] nh NodeHandle
                     */
                    explicit Reader(const ::ros::NodeHandle &nh)
                    {
                        nh_ = nh;
                    }


                    bool descend(const std::string & child_name)
                    {
                        if (0 == node_stack_.size())
                        {
                            root_name_ = child_name;
                            nh_.getParam(root_name_, root_value_);
                            node_stack_.push_back(&root_value_);
                            return(true);
                        }
                        else
                        {
                            XmlRpc::XmlRpcValue & node = getRawNode();
                            if ((XmlRpc::XmlRpcValue::TypeStruct == node.getType()) && (true == node.hasMember(child_name)))
                            {
                                node_stack_.push_back(   NodeWrapper(  &( node[child_name] )  )   );
                                return (true);
                            }
                            else
                            {
                                return (false);
                            }
                        }
                    }


                    void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    bool getMapEntryNames(std::vector<std::string> &child_names)
                    {
                        XmlRpc::XmlRpcValue selected_node = getRawNode();

                        if(XmlRpc::XmlRpcValue::TypeStruct != selected_node.getType())
                        {
                            return (false);
                        }
                        else
                        {
                            child_names.resize(selected_node.size());

                            std::size_t i = 0;
                            for(XmlRpc::XmlRpcValue::iterator it = selected_node.begin(); it != selected_node.end(); ++it, ++i)
                            {
                                child_names[i] = it->first;
                            }
                            return (true);
                        }
                    }


                    std::size_t startArray()
                    {
                        ARILES_ASSERT(XmlRpc::XmlRpcValue::TypeArray == getRawNode().getType(), "Expected array.");

                        std::size_t size = getRawNode().size();
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


                    #define ARILES_BASIC_TYPE(type) \
                            void readElement(type &element) \
                            { \
                                ARILES_ASSERT(getRawNode().getType() == XmlRpc::XmlRpcValue::TypeInt,\
                                              "Integer type expected."); \
                                int tmp_value = static_cast<int>(getRawNode()); \
                                ARILES_ASSERT(tmp_value <= std::numeric_limits<type>::max() \
                                              && tmp_value >= std::numeric_limits<type>::min(), \
                                              "Value is out of range."); \
                                element = static_cast<type>(tmp_value); \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_SIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                            void readElement(type &element) \
                            { \
                                ARILES_ASSERT(getRawNode().getType() == XmlRpc::XmlRpcValue::TypeInt,\
                                              "Integer type expected."); \
                                int tmp_value = static_cast<int>(getRawNode()); \
                                ARILES_ASSERT(tmp_value >= 0, "Expected positive value."); \
                                ARILES_ASSERT(static_cast<unsigned int>(tmp_value) <= std::numeric_limits<type>::max(), \
                                              "Value is too large."); \
                                element = static_cast<type>(tmp_value); \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element) \
                        { \
                            switch(getRawNode().getType()) \
                            { \
                                case XmlRpc::XmlRpcValue::TypeDouble: \
                                    element = static_cast<double>(getRawNode()); \
                                    break; \
                                case XmlRpc::XmlRpcValue::TypeString: \
                                    element = boost::lexical_cast<double>(  static_cast<std::string>( getRawNode() )  ); \
                                    break; \
                                case XmlRpc::XmlRpcValue::TypeInt: \
                                    element = static_cast<int>(getRawNode()); \
                                    break; \
                                default: \
                                    ARILES_THROW("Could not convert value to type."); \
                                    break; \
                            } \
                        }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_REAL_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    void readElement(std::string &element)
                    {
                        element = static_cast<std::string>(getRawNode());
                    }


                    void readElement(bool &element)
                    {
                        switch(getRawNode().getType())
                        {
                            case XmlRpc::XmlRpcValue::TypeString:
                                element = boost::lexical_cast<bool>(  static_cast<std::string>( getRawNode() )  );
                                break;

                            case XmlRpc::XmlRpcValue::TypeBoolean:
                                element = static_cast<bool>(getRawNode());
                                break;

                            case XmlRpc::XmlRpcValue::TypeInt:
                                element = static_cast<int>(getRawNode()) > 0;
                                break;

                            default:
                                ARILES_THROW("Could not convert value to boolean.");
                                break;
                        }
                    }
            };
        }
    }
}
