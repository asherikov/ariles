/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/graphviz.h>

#include <vector>

#include <boost/lexical_cast.hpp>



namespace ariles2
{
    namespace ns_graphviz
    {
        template <class t_RawNode>
        class Node : public ariles2::Node<t_RawNode>
        {
        public:
            std::string name_;

        public:
            Node(t_RawNode node,
                 const typename ariles2::Node<t_RawNode>::Type type = ariles2::Node<t_RawNode>::GENERIC,
                 const bool compact = false)
              : ariles2::Node<t_RawNode>(node, type, compact){};
            Node(const std::size_t index, const std::size_t size, const bool compact = false)
              : ariles2::Node<t_RawNode>(index, size, compact){};
            Node(t_RawNode node, const std::size_t index, const std::size_t size, const bool compact = false)
              : ariles2::Node<t_RawNode>(node, index, size, compact){};
        };


        typedef Node<std::string> NodeWrapper;
    }  // namespace ns_graphviz
}  // namespace ariles2


namespace ariles2
{
    namespace ns_graphviz
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer
            {
            public:
                std::vector<NodeWrapper> node_stack_;
                /// output file stream
                std::ofstream config_ofs_;

                /// output stream
                std::ostream *output_stream_;


            public:
                explicit Writer(const std::string &file_name)
                {
                    ariles2::write::Visitor::openFile(config_ofs_, file_name);
                    output_stream_ = &config_ofs_;
                }

                explicit Writer(std::ostream &output_stream)
                {
                    output_stream_ = &output_stream;
                }


                void writeNodeAndConnection()
                {
                    ARILES2_TRACE_FUNCTION;

                    const std::size_t stack_size = node_stack_.size();

                    ARILES2_ASSERT(0 < stack_size, "Internal error: stack must contain at least 2 entries.");

                    // node
                    std::string node_name_suffix;
                    if (true == node_stack_.back().isArray())
                    {
                        node_name_suffix = "_";
                        node_name_suffix += boost::lexical_cast<std::string>(node_stack_.back().index_);
                    }

                    *output_stream_                                          //
                            << node_stack_.back().node_ << node_name_suffix  //
                            << "[label = \""                                 //
                            << node_stack_.back().name_ << node_name_suffix  //
                            << "\"];\n";


                    // connection
                    if (stack_size > 1)
                    {
                        std::string parent_node_name_suffix;
                        if (true == node_stack_[stack_size - 2].isArray())
                        {
                            parent_node_name_suffix = "_";
                            parent_node_name_suffix +=
                                    boost::lexical_cast<std::string>(node_stack_[stack_size - 2].index_);
                        }

                        *output_stream_                                                          //
                                << node_stack_[stack_size - 2].node_ << parent_node_name_suffix  //
                                << "->"                                                          //
                                << node_stack_.back().node_ << node_name_suffix << ";\n";
                    }
                }
            };
        }  // namespace impl
    }      // namespace ns_graphviz
}  // namespace ariles2


namespace ariles2
{
    namespace ns_graphviz
    {
        Writer::Writer(const std::string &file_name)
        {
            impl_ = ImplPtr(new Impl(file_name));
        }


        Writer::Writer(std::ostream &output_stream)
        {
            impl_ = ImplPtr(new Impl(output_stream));
        }


        const serialization::Features &Writer::getSerializationFeatures() const
        {
            static serialization::Features parameters(
                    serialization::Features::SLOPPY_MAPS_SUPPORTED | serialization::Features::SLOPPY_PAIRS_SUPPORTED);
            return (parameters);
        }


        void Writer::flush()
        {
            impl_->output_stream_->flush();
        }


        void Writer::startRoot(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == name.empty())
            {
                impl_->node_stack_.push_back(std::string("ariles"));
            }
            else
            {
                impl_->node_stack_.push_back(name);
            }
            impl_->node_stack_.back().name_ = impl_->node_stack_.back().node_;
            *impl_->output_stream_                                          //
                    << "digraph graph_" << impl_->node_stack_.back().node_  //
                    << " {\n"                                               //
                    << "rankdir=\"LR\"\n"                                   //
                    << impl_->node_stack_.back().name_ << ";\n";
        }


        void Writer::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            *impl_->output_stream_ << "}\n";
        }


        void Writer::descend(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == impl_->node_stack_.back().isArray())
            {
                std::string node = impl_->node_stack_.back().node_;
                node += "_";
                node += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_ + 1);
                node += "_";
                node += name;
                impl_->node_stack_.push_back(NodeWrapper(node));
            }
            else
            {
                impl_->node_stack_.push_back(impl_->node_stack_.back().node_ + "_" + name);
            }
            impl_->node_stack_.back().name_ = name;
        }


        void Writer::ascend()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


        void Writer::startMap(const std::size_t /*num_entries*/)
        {
            ARILES2_TRACE_FUNCTION;
            impl_->writeNodeAndConnection();
        }

        void Writer::endMap()
        {
            ARILES2_TRACE_FUNCTION;
        }


        void Writer::startArray(const std::size_t size, const bool compact)
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(false == impl_->node_stack_.empty(), "Internal error: empty stack.");

            impl_->writeNodeAndConnection();

            const std::string name = impl_->node_stack_.back().name_;
            if (true == impl_->node_stack_.back().isArray())
            {
                std::string node = impl_->node_stack_.back().node_;
                node += "_";
                node += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_ + 1);
                impl_->node_stack_.push_back(NodeWrapper(node, 0, size, compact));
            }
            else
            {
                impl_->node_stack_.push_back(NodeWrapper(impl_->node_stack_.back().node_, 0, size));
            }
            impl_->node_stack_.back().name_ = name;
        }

        void Writer::shiftArray()
        {
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: array expected.");
            ++impl_->node_stack_.back().index_;
        }

        void Writer::endArray()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }



#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &)                                                                            \
    {                                                                                                                  \
        impl_->writeNodeAndConnection();                                                                               \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_graphviz
}  // namespace ariles2