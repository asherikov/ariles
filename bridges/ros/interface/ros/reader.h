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
            class ARILES_VISIBILITY_ATTRIBUTE Reader : public ariles::ReaderBase
            {
                protected:
                    typedef ariles::Node<XmlRpc::XmlRpcValue> NodeWrapper;


                protected:
                    /// Stack of nodes.
                    std::vector<NodeWrapper>    node_stack_;

                    std::string                 root_name_;
                    XmlRpc::XmlRpcValue         root_value_;

                    ::ros::NodeHandle nh_;


                protected:
                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
                    XmlRpc::XmlRpcValue & getRawNode(const std::size_t depth)
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


                    XmlRpc::XmlRpcValue & getRawNode()
                    {
                        return(getRawNode(node_stack_.size()-1));
                    }


                    std::size_t getMapSize()
                    {
                        return (getRawNode().size());
                    }


                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] file_name
                     */
                    explicit Reader(const ::ros::NodeHandle &nh)
                    {
                        nh_ = nh;
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
                        if (0 == node_stack_.size())
                        {
                            root_name_ = child_name;
                            nh_.getParam(root_name_, root_value_);
                            node_stack_.push_back(&root_value_);
                        }
                        else
                        {
                            XmlRpc::XmlRpcValue & node = getRawNode();
                            ARILES_ASSERT(XmlRpc::XmlRpcValue::TypeStruct == node.getType(), "Expected struct.");
                            ARILES_ASSERT(true == node.hasMember(child_name), std::string("Child not found: ") + child_name);
                            node_stack_.push_back(   NodeWrapper(  &( node[child_name] )  )   );
                        }
                        return(true);
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
                        if (node_stack_.back().isArray())
                        {
                            ++node_stack_.back().index_;
                        }
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
                                case XmlRpc::XmlRpcValue::Typetype: \
                                    element = static_cast<type>(getRawNode()); \
                                    break; \
                                case XmlRpc::XmlRpcValue::TypeString: \
                                    element = boost::lexical_cast<type>(  static_cast<std::string>( getRawNode() )  ); \
                                    break; \
                                case XmlRpc::XmlRpcValue::TypeInt: \
                                    element = static_cast<int>(getRawNode()); \
                                    break; \
                                default: \
                                    ARILES_THROW_MSG("Could not convert value to type."); \
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
                                ARILES_THROW_MSG("Could not convert value to boolean.");
                                break;
                        }
                    }
            };
        }
    }
}
