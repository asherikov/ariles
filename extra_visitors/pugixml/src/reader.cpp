/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "common.h"

namespace ariles2
{
    namespace ns_pugixml
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader
            {
            public:
                pugi::xml_document document_;

                std::vector<NodeWrapper> node_stack_;


            public:
                /**
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 */
                pugi::xml_node &getRawNode()
                {
                    return (node_stack_.back().node_);
                }
            };
        }  // namespace impl
    }      // namespace ns_pugixml
}  // namespace ariles2


namespace ariles2
{
    namespace ns_pugixml
    {
        Reader::Reader(const std::string &file_name)
        {
            impl_ = std::make_shared<impl::Reader>();

            const pugi::xml_parse_result result = impl_->document_.load_file(file_name.c_str(), pugi::parse_minimal);
            ARILES2_ASSERT(result, std::string("Parsing of '") + file_name + "' failed: " + result.description());
            impl_->node_stack_.push_back(impl_->document_);  // NOLINT
        }


        Reader::Reader(std::istream &input_stream)
        {
            impl_ = std::make_shared<impl::Reader>();

            const pugi::xml_parse_result result = impl_->document_.load(input_stream, pugi::parse_minimal);
            ARILES2_ASSERT(result, std::string("Parsing failed: ") + result.description());
            impl_->node_stack_.push_back(impl_->document_);  // NOLINT
        }


        bool Reader::startMapEntry(const std::string &child_name)
        {
            const pugi::xml_node child = impl_->getRawNode().child(child_name.c_str());

            if (NULL != child)
            {
                impl_->node_stack_.emplace_back(child);
                return (true);
            }

            const pugi::xml_attribute attribute = impl_->getRawNode().attribute(child_name.c_str());
            if (NULL != attribute)
            {
                const pugi::xml_node new_child = impl_->getRawNode().append_child(child_name.c_str());
                new_child.text() = attribute.value();
                impl_->node_stack_.emplace_back(new_child);
                return (true);
            }

            return (false);
        }


        void Reader::endMapEntry()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType /*limit_type*/,
                const std::size_t /*min*/,
                const std::size_t /*max*/)
        {
            const pugi::xml_node child = impl_->getRawNode().first_child();
            if (NULL != child)
            {
                impl_->node_stack_.emplace_back(child, NodeWrapper::ITERATED_MAP);
                return (true);
            }
            return (false);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            if (NULL != impl_->getRawNode())
            {
                entry_name = impl_->getRawNode().name();
                return (true);
            }
            return (false);
        }

        void Reader::endIteratedMapElement()
        {
            const pugi::xml_node node = impl_->getRawNode();
            if (NULL != node)
            {
                impl_->getRawNode() = node.next_sibling();
            }
        }

        void Reader::endIteratedMap()
        {
            ARILES2_ASSERT(!impl_->getRawNode(), "End of iterated map has not been reached.");
            impl_->node_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            std::size_t size = 0;
            const pugi::xml_node node = impl_->getRawNode();
            for (pugi::xml_node child = node.child("item"); NULL != child; child = child.next_sibling("item"), ++size)
            {
            }

            if (size > 0)
            {
                impl_->node_stack_.emplace_back(node.child("item"), 0, size);
            }
            else
            {
                // if there are no 'item' childs try to iterate
                // over childs with the same name in the parent
                // node
                for (pugi::xml_node child = impl_->getRawNode(); NULL != child;
                     child = child.next_sibling(child.name()), ++size)
                {
                }
                impl_->node_stack_.emplace_back(impl_->getRawNode(), 0, size);
            }

            return (size);
        }


        void Reader::startArrayElement()
        {
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: namevalue.has more elements than expected.");
        }


        void Reader::endArrayElement()
        {
            ARILES2_ASSERT(impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            impl_->node_stack_.back().node_ = impl_->getRawNode().next_sibling(impl_->getRawNode().name());
            ++impl_->node_stack_.back().index_;
        }


        void Reader::endArray()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::startRoot(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            if (name.empty())
            {
                return (startMapEntry("ariles"));
            }
            return (startMapEntry(name));
        }

        void Reader::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            endMapEntry();
        }


        void Reader::readElement(std::string &element)
        {
            element = impl_->getRawNode().text().as_string();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        ARILES2_ASSERT(not impl_->getRawNode().text().empty(), "Empty integer elements are not allowed.");             \
        element = boost::lexical_cast<type>(impl_->getRawNode().text().as_string());                                   \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_pugixml
}  // namespace ariles2
