/**
    @file
    @author Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <string>
#include <vector>
#include <utility>
#include <boost/lexical_cast.hpp>


namespace ariles
{
    namespace bridge
    {
        namespace array
        {
            typedef std::pair<std::string, double>  NameValuePair;


            template<class t_NameValuePair>
            class ARILES_VISIBILITY_ATTRIBUTE NameValuePairHandler
            {
            };


            template<>
            class ARILES_VISIBILITY_ATTRIBUTE NameValuePairHandler< NameValuePair >
            {
                public:
                    static inline std::string & name(NameValuePair & pair)
                    {
                        return (pair.first);
                    }

                    static inline double & value(NameValuePair & pair)
                    {
                        return (pair.second);
                    }
            };


            /**
             * @brief Configuration writer class
             */
            template<class t_NameValuePair>
            class ARILES_VISIBILITY_ATTRIBUTE GenericWriter : public ariles::WriterBase
            {
                protected:
                    typedef ariles::Node< std::string > NodeWrapper;


                protected:
                    std::vector<NodeWrapper>        node_stack_;
                    std::size_t                     reserve_;

                    std::vector<t_NameValuePair>    buffer_name_value_pairs_;

                    bool                            initialize_structure_;


                public:
                    std::vector<t_NameValuePair>    *name_value_pairs_;
                    std::size_t                     index_;


                protected:
                    void expand()
                    {
                        if (index_ == name_value_pairs_->size())
                        {
                            name_value_pairs_->resize(name_value_pairs_->size() + 1);
                        }
                    }

                    void expandReserve(const std::size_t size)
                    {
                        reserve_ += size;
                        name_value_pairs_->reserve(reserve_);
                    }

                    void clear()
                    {
                        name_value_pairs_->clear();
                    }


                public:
                    explicit GenericWriter(const std::size_t reserve = 0)
                    {
                        name_value_pairs_ = &buffer_name_value_pairs_;

                        if (reserve > 0)
                        {
                            expandReserve(reserve);
                        }
                        reset();
                    }


                    explicit GenericWriter(std::vector<t_NameValuePair> *name_value_pairs, const std::size_t reserve = 0)
                    {
                        name_value_pairs_ = name_value_pairs;

                        if (reserve > 0)
                        {
                            expandReserve(reserve);
                        }
                        reset();
                    }


                    const BridgeFlags &getBridgeFlags() const
                    {
                        static BridgeFlags parameters;
                        return (parameters);
                    }


                    void flush()
                    {
                    }


                    void reset(const bool initialize_structure = true)
                    {
                        if (true == initialize_structure)
                        {
                            clear();
                        }
                        initialize_structure_ = initialize_structure;
                        index_ = 0;
                        reserve_ = 0;
                    }


                    virtual void descend(const std::string & map_name)
                    {
                        if (true == initialize_structure_)
                        {
                            if (0 == node_stack_.size())
                            {
                                node_stack_.push_back(map_name);
                            }
                            else
                            {
                                if (true == node_stack_.back().isArray())
                                {
                                    std::string node = node_stack_.back().node_;
                                    node += "{";
                                    node += boost::lexical_cast<std::string>(node_stack_.back().index_);
                                    node += "}.";
                                    node += map_name;
                                    node_stack_.push_back(node);
                                }
                                else
                                {
                                    node_stack_.push_back(node_stack_.back().node_ + "." + map_name);
                                }
                            }
                        }
                    }

                    virtual void ascend()
                    {
                        if (true == initialize_structure_)
                        {
                            node_stack_.pop_back();
                        }
                    }


                    virtual void startMap(const std::size_t num_entries)
                    {
                        if (true == initialize_structure_)
                        {
                            expandReserve(num_entries);
                        }
                    }

                    virtual void endMap() {}


                    virtual void startArray(const std::size_t size, const bool compact = false)
                    {
                        if (true == initialize_structure_)
                        {
                            expandReserve(size);
                            if (true == node_stack_.back().isArray())
                            {
                                std::string node = node_stack_.back().node_;
                                node += "_";
                                node += boost::lexical_cast<std::string>(node_stack_.back().index_);
                                node_stack_.push_back(NodeWrapper(node, 0, size, compact));
                            }
                            else
                            {
                                node_stack_.push_back(NodeWrapper(node_stack_.back().node_, 0, size));
                            }
                        }
                    }

                    virtual void shiftArray()
                    {
                        if (true == initialize_structure_)
                        {
                            ARILES_ASSERT(true == node_stack_.back().isArray(), "Internal error: array expected.");
                            ++node_stack_.back().index_;
                        }
                    }

                    virtual void endArray()
                    {
                        if (true == initialize_structure_)
                        {
                            node_stack_.pop_back();
                        }
                    }


                    #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element) \
                            { \
                                expand(); \
                                if (true == initialize_structure_) \
                                { \
                                    NameValuePairHandler<t_NameValuePair>::name((*name_value_pairs_)[index_]) = node_stack_.back().node_; \
                                    if (true == node_stack_.back().isArray()) \
                                    { \
                                        NameValuePairHandler<t_NameValuePair>::name((*name_value_pairs_)[index_]) += "_"; \
                                        NameValuePairHandler<t_NameValuePair>::name((*name_value_pairs_)[index_]) += boost::lexical_cast<std::string>(node_stack_.back().index_); \
                                    } \
                                } \
                                NameValuePairHandler<t_NameValuePair>::value((*name_value_pairs_)[index_]) = element; \
                                ++index_; \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_NUMERIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    void writeElement(const std::string & /*element*/)
                    {
                    }
            };


            typedef GenericWriter<NameValuePair> Writer;
        }
    }
}
