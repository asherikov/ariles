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
            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer : public ariles::WriterBase
            {
                protected:
                    typedef ariles::Node< std::string > NodeWrapper;


                protected:
                    std::vector<NodeWrapper>    node_stack_;
                    std::size_t                 reserve_;

                    std::vector<std::string>    buffer_names_;
                    std::vector<double>         buffer_values_;

                    std::string                 prefix_;

                    bool                        preserve_structure_;


                public:
                    std::vector<std::string>    *names_;
                    std::vector<double>         *values_;
                    std::size_t                 index_;


                protected:
                    void expand()
                    {
                        if (index_ == names_->size())
                        {
                            names_->resize(names_->size() + 1);
                            values_->resize(values_->size() + 1);
                        }
                    }

                    void expandReserve(const std::size_t size)
                    {
                        if (false == preserve_structure_)
                        {
                            reserve_ += size;
                            names_->reserve(reserve_);
                            values_->reserve(reserve_);
                        }
                    }

                    void clear()
                    {
                        names_->clear();
                        values_->clear();
                    }


                public:
                    explicit Writer(const std::size_t reserve = 0, const std::string &prefix = "")
                    {
                        names_ = &buffer_names_;
                        values_ = &buffer_values_;
                        prefix_ = prefix;

                        if (reserve > 0)
                        {
                            expandReserve(reserve);
                        }
                        reset();
                    }


                    explicit Writer(std::vector<std::string> *names, std::vector<double> *values, const std::size_t reserve, const std::string &prefix = "")
                    {
                        names_ = names;
                        values_ = values;
                        prefix_ = prefix;

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


                    void reset(const bool preserve_structure = false)
                    {
                        if (false == preserve_structure)
                        {
                            clear();
                        }
                        preserve_structure_ = preserve_structure;
                        index_ = 0;
                        reserve_ = 0;
                    }


                    virtual void descend(const std::string & map_name)
                    {
                        if (0 == node_stack_.size())
                        {
                            node_stack_.push_back(prefix_ + map_name);
                        }
                        else
                        {
                            if (true == preserve_structure_)
                            {
                                node_stack_.push_back(NodeWrapper(""));
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
                        node_stack_.pop_back();
                    }


                    virtual void startMap(const std::size_t num_entries)
                    {
                        expandReserve(num_entries);
                    }

                    virtual void endMap() {}


                    virtual void startArray(const std::size_t size, const bool compact = false)
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

                    virtual void shiftArray()
                    {
                        ARILES_ASSERT(true == node_stack_.back().isArray(), "Internal error: array expected.");
                        ++node_stack_.back().index_;
                    }

                    virtual void endArray()
                    {
                        node_stack_.pop_back();
                    }


                    #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element) \
                            { \
                                expand(); \
                                if (false == preserve_structure_) \
                                { \
                                    (*names_)[index_] = node_stack_.back().node_; \
                                    if (true == node_stack_.back().isArray()) \
                                    { \
                                        (*names_)[index_] += "_"; \
                                        (*names_)[index_] += boost::lexical_cast<std::string>(node_stack_.back().index_); \
                                    } \
                                } \
                                (*values_)[index_] = element; \
                                ++index_; \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_NUMERIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    void writeElement(const std::string & /*element*/)
                    {
                    }
            };
        }
    }
}
