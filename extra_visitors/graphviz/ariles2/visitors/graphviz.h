/**
    @file
    @author Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup graphviz Graphviz

@brief Generates diagrams ('.dot' files) representing class contents.

The following example is generated using classes defined in @ref ariles_diagram.h
@image html graphviz_example.svg
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_graphviz

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/write.h>

#include <string>


namespace ariles2
{
    namespace ns_graphviz
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Visitor;
        }


        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public write::Parameters
        {
        public:
            class ARILES2_VISIBILITY_ATTRIBUTE NodeOptions
            {
            public:
                std::string id_;
                std::string label_;
                std::string options_;

            public:
                NodeOptions()
                {
                    id_ = "";
                    label_ = "";
                    options_ = "";
                }

                NodeOptions(const std::string &id)
                {
                    id_ = id;
                    label_ = id_;
                    options_ = "";
                }

                NodeOptions(const std::string &id, const std::string &label)
                {
                    id_ = id;
                    label_ = label;
                    options_ = "";
                }

                NodeOptions(const std::string &id, const std::string &label, const std::string &options)
                {
                    id_ = id;
                    label_ = label;
                    options_ = options;
                }
            };


        public:
            std::string graph_options_;
            std::string node_options_;

        public:
            Parameters(const bool override_parameters = false) : write::Parameters(override_parameters)
            {
                graph_options_ = "rankdir=\"LR\"\n";
                node_options_ = "";
                sloppy_maps_ = true;
                sloppy_pairs_ = true;
                compact_arrays_ = true;
            }

            virtual NodeOptions getDefaultNodeOptions(const std::string &id, const std::string &label) const
            {
                return (NodeOptions(id, label, node_options_));
            }
            virtual NodeOptions getArilesNodeOptions(const std::string &id, const std::string &label) const
            {
                return (NodeOptions(id, label, node_options_));
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public serialization::PIMPLVisitor<write::VisitorBase<Visitor, Parameters>, impl::Visitor>
        {
        protected:
            void startMap(const Parameters &, const Parameters::NodeOptions &);
            std::string getDefaultNodeId() const;
            std::string getDefaultNodeLabel() const;


        public:
            explicit Visitor(const std::string &file_name);
            explicit Visitor(std::ostream &output_stream);


            void flush();

            void startRoot(const std::string &name, const Parameters &);
            void endRoot(const std::string &name);

            void startMap(const Parameters &, const std::size_t num_entries);
            void startMapEntry(const std::string &map_name);
            void endMapEntry();

            void startArray(const std::size_t size, const bool compact = false);
            void endArrayElement();
            void endArray();


            template <class t_Ariles>
            const Parameters getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }

            template <class t_Entry>
            void startMap(t_Entry &, const Parameters &parameters)
            {
                startMap(parameters, parameters.getArilesNodeOptions(getDefaultNodeId(), getDefaultNodeLabel()));
            }

            using write::VisitorBase<Visitor, Parameters>::startMap;


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element, const Parameters &param);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)
            ARILES2_MACRO_SUBSTITUTE(ARILES2_COMPLEX_NUMBER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::ConstBase<Visitor>
        {
        };


#define ARILES2_NAMED_ENTRY_graphviz(v, entry, name) visitor.visitMapEntry(entry, #name, parameters);
#define ARILES2_PARENT_graphviz(v, entry)
#define ARILES2_VISIT_graphviz                                                                                         \
    template <class t_Visitor>                                                                                         \
    void arilesVisit(                                                                                                  \
            t_Visitor &visitor,                                                                                        \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::graphviz::Visitor, t_Visitor)) const                                      \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        arilesVisitParents(visitor, parameters);                                                                       \
        ARILES2_ENTRIES(graphviz)                                                                                      \
    }

#define ARILES2_METHODS_graphviz ARILES2_METHODS(graphviz, ARILES2_EMPTY_MACRO, const)
#define ARILES2_BASE_METHODS_graphviz ARILES2_BASE_METHODS(graphviz)
    }  // namespace ns_graphviz
}  // namespace ariles2


namespace ariles2
{
    /**
     * @brief Graphviz visitor.
     * @ingroup graphviz
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE graphviz
    {
        using Parameters = ns_graphviz::Parameters;
        using Visitor = ns_graphviz::Visitor;
        using Writer = ns_graphviz::Visitor;
        using Base = ns_graphviz::Base;
    };

    /// @ingroup graphviz
    using Graphviz = graphviz::Visitor;
}  // namespace ariles2
