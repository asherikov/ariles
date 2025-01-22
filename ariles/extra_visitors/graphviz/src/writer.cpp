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
            using Base = serialization::Node<std::string>;

        public:
            std::string actual_id_;
            std::string label_;


        public:
            explicit NodeWrapper(const std::string &node, const Base::Type type = Base::Type::GENERIC)
              : Base(node, type)
            {
                label_ = node;
                actual_id_ = node;
            }

            explicit NodeWrapper(
                    const std::string &node,
                    const std::string &label,
                    const Base::Type type = Base::Type::GENERIC)
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
            class Visitor : public serialization::NodeStackBase<NodeWrapper>, public write::FileVisitorImplementation
            {
            public:
                std::set<std::string> all_ids_;
                const Parameters *parameters_;

                const std::string separator_ = "_";


            public:
                template <class... t_Args>
                explicit Visitor(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                }


                void clear()
                {
                    all_ids_.clear();
                    node_stack_.clear();
                }


                void writeNodeAndConnection(const Parameters::NodeOptions &node_options)
                {
                    CPPUT_TRACE_FUNCTION;

                    const std::size_t stack_size = node_stack_.size();

                    CPPUT_ASSERT(0 < stack_size, "Internal error: stack must contain at least 2 entries.");

                    // node
                    back().actual_id_ = node_options.id_;

                    if (all_ids_.insert(back().actual_id_).second)
                    {
                        *output_stream_ << node_options.id_;
                        *output_stream_ << "[";
                        if (not node_options.label_.empty())
                        {
                            *output_stream_ << "label=\"" << node_options.label_ << "\"";
                        }
                        if (not node_options.options_.empty())
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
                                << back().actual_id_ << ";\n";
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
            makeImplPtr(file_name);
        }


        Visitor::Visitor(std::ostream &output_stream)
        {
            makeImplPtr(output_stream);
        }


        void Visitor::flush()
        {
            impl_->output_stream_->flush();
        }


        void Visitor::startRoot(const std::string &name, const Parameters &parameters)
        {
            CPPUT_TRACE_FUNCTION;
            impl_->clear();
            impl_->parameters_ = &parameters;
            if (name.empty())
            {
                impl_->emplace("ariles");
            }
            else
            {
                impl_->emplace(name);
            }
            *impl_->output_stream_                              //
                    << "digraph graph_" << impl_->back().node_  //
                    << " {\n"                                   //
                    << parameters.graph_options_;               //
        }


        void Visitor::endRoot(const std::string & /*name*/)
        {
            CPPUT_TRACE_FUNCTION;
            *impl_->output_stream_ << "}\n";
        }

        std::string Visitor::getDefaultNodeId() const
        {
            if (impl_->back().isArray())
            {
                return (impl_->back().node_ + "_" + boost::lexical_cast<std::string>(impl_->back().index_));
            }
            return (impl_->back().node_);
        }

        std::string Visitor::getDefaultNodeLabel() const
        {
            if (impl_->back().isArray())
            {
                return (impl_->back().label_ + "_" + boost::lexical_cast<std::string>(impl_->back().index_));
            }
            return (impl_->back().label_);
        }

        void Visitor::startMap(const Parameters &parameters, const Parameters::NodeOptions &node_options)
        {
            CPPUT_TRACE_FUNCTION;
            if (not impl_->parameters_->override_parameters_)
            {
                impl_->parameters_ = &parameters;
            }
            impl_->writeNodeAndConnection(node_options);
        }

        void Visitor::startMap(const Parameters &parameters, const std::size_t /*num_entries*/)
        {
            CPPUT_TRACE_FUNCTION;
            if (not impl_->parameters_->override_parameters_)
            {
                impl_->parameters_ = &parameters;
            }
            impl_->writeNodeAndConnection(
                    impl_->parameters_->getDefaultNodeOptions(getDefaultNodeId(), getDefaultNodeLabel()));
        }

        void Visitor::startMapEntry(const std::string &name)
        {
            CPPUT_TRACE_FUNCTION;
            if (impl_->back().isArray())
            {
                impl_->emplace(
                        impl_->concatWithNode(
                                impl_->separator_,
                                boost::lexical_cast<std::string>(impl_->back().index_),
                                impl_->separator_,
                                name),
                        name);
            }
            else
            {
                impl_->emplace(impl_->concatWithNode(impl_->separator_, name), name);
            }
        }

        void Visitor::endMapEntry()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
        }


        void Visitor::startArray(const std::size_t size, const bool compact)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(not impl_->empty(), "Internal error: empty stack.");

            if (size > 0 || not compact)
            {
                impl_->writeNodeAndConnection(
                        impl_->parameters_->getDefaultNodeOptions(getDefaultNodeId(), getDefaultNodeLabel()));
            }

            if (impl_->back().isArray())
            {
                const std::string index = boost::lexical_cast<std::string>(impl_->back().index_);
                impl_->emplace(
                        impl_->concatWithNode(impl_->separator_, index),
                        cpput::concat::simple(impl_->back().label_, impl_->separator_, index),
                        0,
                        size);
            }
            else
            {
                impl_->emplace(impl_->back().node_, impl_->back().label_, 0, size);
            }
        }

        void Visitor::endArrayElement()
        {
            impl_->shiftArray();
        }

        void Visitor::endArray()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Visitor::writeElement(const type &, const Parameters &)                                                       \
    {                                                                                                                  \
        impl_->writeNodeAndConnection(                                                                                 \
                impl_->parameters_->getDefaultNodeOptions(getDefaultNodeId(), getDefaultNodeLabel()));                 \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)
        CPPUT_MACRO_SUBSTITUTE(ARILES2_COMPLEX_NUMBER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_graphviz
}  // namespace ariles2
