/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <limits>
#include <iomanip>
#include <vector>
#include <boost/lexical_cast.hpp>

namespace ariles
{
    namespace bridge
    {
        namespace octave
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

                    /// output file stream
                    std::ofstream   config_ofs_;

                    /// output stream
                    std::ostream    *output_stream_;


                protected:
                    /**
                     * @brief Initialize emitter
                     */
                    void initEmitter()
                    {
                        *output_stream_ << std::setprecision(std::numeric_limits<double>::digits10);
                    }


                public:
                    explicit Writer(const std::string& file_name)
                    {
                        WriterBase::openFile(config_ofs_, file_name);
                        output_stream_ = &config_ofs_;
                        initEmitter();
                    }


                    explicit Writer(std::ostream& output_stream)
                    {
                        output_stream_ = &output_stream;
                        initEmitter();
                    }


                    const BridgeFlags &getBridgeFlags() const
                    {
                        static BridgeFlags parameters(BridgeFlags::NATIVE_MATRIX_SUPPORTED);
                        return (parameters);
                    }


                    void flush()
                    {
                        output_stream_->flush();
                    }


                    virtual void initRoot() {};


                    virtual void descend(const std::string & map_name)
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
                                node += boost::lexical_cast<std::string>(node_stack_.back().index_ + 1);
                                node += "}.";
                                node += map_name;
                                node_stack_.push_back(NodeWrapper(node));
                            }
                            else
                            {
                                node_stack_.push_back(node_stack_.back().node_ + "." + map_name);
                            }
                        }
                    }

                    virtual void ascend()
                    {
                        node_stack_.pop_back();
                    }


                    virtual void startMap(const std::size_t /*num_entries*/) {}
                    virtual void endMap() {}


                    virtual void startArray(const std::size_t size, const bool compact = false)
                    {
                        if (true == node_stack_.back().isArray())
                        {
                            std::string node = node_stack_.back().node_;
                            node += "{";
                            node += boost::lexical_cast<std::string>(node_stack_.back().index_ + 1);
                            node += "}";
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


                    void startMatrix(const bool compact = false)
                    {
                        std::string node = node_stack_.back().node_;
                        if (compact)
                        {
                            node += " = [";
                        }
                        else
                        {
                            node += " = [...\n";
                        }

                        node_stack_.push_back(NodeWrapper(node, NodeWrapper::MATRIX, compact));
                        *output_stream_ << node_stack_.back().node_;
                    }

                    void startMatrixRow()
                    {
                        node_stack_.back().index_ = 0;
                    }

                    void endMatrixRow()
                    {
                        if (node_stack_.back().isCompact())
                        {
                            *output_stream_ << "; ";
                        }
                        else
                        {
                            *output_stream_ << "; ...\n";
                        }
                    }

                    void endMatrix()
                    {
                        *output_stream_ << "];\n";
                        node_stack_.pop_back();
                    }



                    #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element) \
                            { \
                                if (true == node_stack_.back().isMatrix()) \
                                { \
                                    if (0 != node_stack_.back().index_) \
                                    { \
                                        *output_stream_ << ", "; \
                                    } \
                                    *output_stream_ << element; \
                                    ++node_stack_.back().index_; \
                                } \
                                else \
                                { \
                                    *output_stream_ << node_stack_.back().node_; \
                                    if (true == node_stack_.back().isArray()) \
                                    { \
                                        *output_stream_ << "{" << node_stack_.back().index_ + 1 << "}"; \
                                    } \
                                    *output_stream_ << " = "; \
                                    *output_stream_ << element; \
                                    *output_stream_ << ";\n"; \
                                } \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_NUMERIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    void writeElement(const std::string & element)
                    {
                        *output_stream_ << node_stack_.back().node_;
                        if (true == node_stack_.back().isArray())
                        {
                            *output_stream_ << "{" << node_stack_.back().index_ + 1 << "}";
                        }
                        *output_stream_ << " = '";
                        *output_stream_ << element;
                        *output_stream_ << "';\n";
                    }
            };
        }
    }
}
