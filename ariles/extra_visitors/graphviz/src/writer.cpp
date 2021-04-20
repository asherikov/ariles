/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/graphviz.h>

#include <vector>
#include <utility>
#include <set>

#include <boost/lexical_cast.hpp>



namespace ariles2
{
    namespace ns_graphviz
    {
        class NodeWrapper : public serialization::Node<std::string>
        {
        public:
            typedef serialization::Node<std::string> Base;

        public:
            std::string actual_id_;
            std::string label_;


        public:
            explicit NodeWrapper(const std::string &node, const Base::Type type = Base::GENERIC) : Base(node, type)
            {
                label_ = node;
                actual_id_ = node;
            }

            explicit NodeWrapper(
                    const std::string &node,
                    const std::string &label,
                    const Base::Type type = Base::GENERIC)
              : Base(node, type)
            {
                label_ = label;
                actual_id_ = node;
            }

            NodeWrapper(
                    const std::string &node,
                    const std::string &label,
                    const std::size_t index,
                    const std::size_t size)
              : Base(node, index, size)
            {
                label_ = label;
                actual_id_ = node;
            }
        };
    }  // namespace ns_graphviz
}  // namespace ariles2


namespace ariles2
{
    namespace ns_graphviz
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Visitor
            {
            public:
                std::vector<NodeWrapper> node_stack_;
                /// output file stream
                std::ofstream config_ofs_;

                /// output stream
                std::ostream *output_stream_;

                std::set<std::string> all_ids_;
                const Parameters *parameters_;


            public:
                explicit Visitor(const std::string &file_name)
                {
                    ariles2::write::Visitor::openFile(config_ofs_, file_name);
                    output_stream_ = &config_ofs_;
                }

                explicit Visitor(std::ostream &output_stream)
                {
                    output_stream_ = &output_stream;
                }


                void clear()
                {
                    all_ids_.clear();
                    node_stack_.clear();
                }


                void writeNodeAndConnection(const Parameters::NodeOptions &node_options)
                {
                    ARILES2_TRACE_FUNCTION;

                    const std::size_t stack_size = node_stack_.size();

                    ARILES2_ASSERT(0 < stack_size, "Internal error: stack must contain at least 2 entries.");

                    // node
                    node_stack_.back().actual_id_ = node_options.id_;

                    if (true == all_ids_.insert(node_stack_.back().actual_id_).second)
                    {
                        *output_stream_ << node_options.id_;
                        *output_stream_ << "[";
                        if (false == node_options.label_.empty())
                        {
                            *output_stream_ << "label=\"" << node_options.label_ << "\"";
                        }
                        if (false == node_options.options_.empty())
                        {
                            *output_stream_ << "," << node_options.options_;
                        }
                        *output_stream_ << "];\n";
                    }

                    // connection
                    if (stack_size > 1)
                    {
                        *output_stream_                                    //
                                << node_stack_[stack_size - 2].actual_id_  //
                                << "->"                                    //
                                << node_stack_.back().actual_id_ << ";\n";
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
        Visitor::Visitor(const std::string &file_name)
        {
            impl_ = ImplPtr(new Impl(file_name));
        }


        Visitor::Visitor(std::ostream &output_stream)
        {
            impl_ = ImplPtr(new Impl(output_stream));
        }


        void Visitor::flush()
        {
            impl_->output_stream_->flush();
        }


        void Visitor::startRoot(const std::string &name, const Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;
            impl_->clear();
            impl_->parameters_ = &parameters;
            if (true == name.empty())
            {
                impl_->node_stack_.push_back(NodeWrapper("ariles"));
            }
            else
            {
                impl_->node_stack_.push_back(NodeWrapper(name));
            }
            *impl_->output_stream_                                          //
                    << "digraph graph_" << impl_->node_stack_.back().node_  //
                    << " {\n"                                               //
                    << parameters.graph_options_;                           //
        }


        void Visitor::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            *impl_->output_stream_ << "}\n";
        }

        std::string Visitor::getDefaultNodeId() const
        {
            if (true == impl_->node_stack_.back().isArray())
            {
                return (impl_->node_stack_.back().node_ + "_"
                        + boost::lexical_cast<std::string>(impl_->node_stack_.back().index_));
            }
            return (impl_->node_stack_.back().node_);
        }

        std::string Visitor::getDefaultNodeLabel() const
        {
            if (true == impl_->node_stack_.back().isArray())
            {
                return (impl_->node_stack_.back().label_ + "_"
                        + boost::lexical_cast<std::string>(impl_->node_stack_.back().index_));
            }
            return (impl_->node_stack_.back().label_);
        }

        void Visitor::startMap(const Parameters &parameters, const Parameters::NodeOptions &node_options)
        {
            ARILES2_TRACE_FUNCTION;
            if (false == impl_->parameters_->override_parameters_)
            {
                impl_->parameters_ = &parameters;
            }
            impl_->writeNodeAndConnection(node_options);
        }

        void Visitor::startMap(const Parameters &parameters, const std::size_t /*num_entries*/)
        {
            ARILES2_TRACE_FUNCTION;
            if (false == impl_->parameters_->override_parameters_)
            {
                impl_->parameters_ = &parameters;
            }
            impl_->writeNodeAndConnection(
                    impl_->parameters_->getDefaultNodeOptions(getDefaultNodeId(), getDefaultNodeLabel()));
        }

        void Visitor::startMapEntry(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == impl_->node_stack_.back().isArray())
            {
                std::string node = impl_->node_stack_.back().node_;
                node += "_";
                node += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_);
                node += "_";
                node += name;
                impl_->node_stack_.push_back(NodeWrapper(node, name));
            }
            else
            {
                impl_->node_stack_.push_back(NodeWrapper(impl_->node_stack_.back().node_ + "_" + name, name));
            }
        }

        void Visitor::endMapEntry()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


        void Visitor::startArray(const std::size_t size, const bool compact)
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(false == impl_->node_stack_.empty(), "Internal error: empty stack.");

            if (size > 0 || false == compact)
            {
                impl_->writeNodeAndConnection(
                        impl_->parameters_->getDefaultNodeOptions(getDefaultNodeId(), getDefaultNodeLabel()));
            }

            if (true == impl_->node_stack_.back().isArray())
            {
                std::string node = impl_->node_stack_.back().node_;
                std::string label = impl_->node_stack_.back().label_;
                node += "_";
                node += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_);
                label += "_";
                label += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_);
                impl_->node_stack_.push_back(NodeWrapper(node, label, 0, size));
            }
            else
            {
                impl_->node_stack_.push_back(
                        NodeWrapper(impl_->node_stack_.back().node_, impl_->node_stack_.back().label_, 0, size));
            }
        }

        void Visitor::endArrayElement()
        {
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: array expected.");
            ++impl_->node_stack_.back().index_;
        }

        void Visitor::endArray()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->node_stack_.pop_back();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Visitor::writeElement(const type &, const Parameters &)                                                       \
    {                                                                                                                  \
        impl_->writeNodeAndConnection(                                                                                 \
                impl_->parameters_->getDefaultNodeOptions(getDefaultNodeId(), getDefaultNodeLabel()));                 \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)
        ARILES2_MACRO_SUBSTITUTE(ARILES2_COMPLEX_NUMBER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_graphviz
}  // namespace ariles2
