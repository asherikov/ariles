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
            impl_ = ImplPtr(new Impl());

            pugi::xml_parse_result result = impl_->document_.load_file(file_name.c_str(), pugi::parse_minimal);
            ARILES2_ASSERT(
                    true == result, std::string("Parsing of '") + file_name + "' failed: " + result.description());
            impl_->node_stack_.push_back(impl_->document_);
        }


        Reader::Reader(std::istream &input_stream)
        {
            impl_ = ImplPtr(new Impl());

            pugi::xml_parse_result result = impl_->document_.load(input_stream, pugi::parse_minimal);
            ARILES2_ASSERT(true == result, std::string("Parsing failed: ") + result.description());
            impl_->node_stack_.push_back(impl_->document_);
        }


        bool Reader::descend(const std::string &child_name)
        {
            const pugi::xml_node child = impl_->getRawNode().child(child_name.c_str());

            if (child)
            {
                impl_->node_stack_.push_back(child);
                return (true);
            }
            else
            {
                const pugi::xml_attribute attribute = impl_->getRawNode().attribute(child_name.c_str());
                if (attribute)
                {
                    pugi::xml_node new_child = impl_->getRawNode().append_child(child_name.c_str());
                    new_child.text() = attribute.value();
                    impl_->node_stack_.push_back(new_child);
                    return (true);
                }
                else
                {
                    return (false);
                }
            }
        }


        void Reader::ascend()
        {
            impl_->node_stack_.pop_back();
        }


        bool Reader::getMapEntryNames(std::vector<std::string> &child_names)
        {
            const pugi::xml_node node = impl_->getRawNode();
            child_names.clear();
            for (pugi::xml_node_iterator it = node.begin(); it != node.end(); ++it)
            {
                child_names.push_back(it->name());
            }
            return (true);
        }


        std::size_t Reader::startArray()
        {
            std::size_t size = 0;
            const pugi::xml_node node = impl_->getRawNode();
            for (pugi::xml_node child = node.child("item"); child; child = child.next_sibling("item"), ++size)
                ;

            if (size > 0)
            {
                impl_->node_stack_.push_back(NodeWrapper(node.child("item"), 0, size));
            }
            else
            {
                // if there are no 'item' childs try to iterate
                // over childs with the same name in the parent
                // node
                for (pugi::xml_node child = impl_->getRawNode(); child;
                     child = child.next_sibling(child.name()), ++size)
                    ;
                impl_->node_stack_.push_back(NodeWrapper(impl_->getRawNode(), 0, size));
            }

            return (size);
        }


        void Reader::startArrayElement()
        {
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
        }


        void Reader::endArrayElement()
        {
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
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
            if (true == name.empty())
            {
                return (descend("ariles"));
            }
            else
            {
                return (descend(name));
            }
        }

        void Reader::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            ascend();
        }


        void Reader::readElement(std::string &element)
        {
            element = impl_->getRawNode().text().as_string();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        ARILES2_ASSERT(false == impl_->getRawNode().text().empty(), "Empty integer elements are not allowed.");        \
        element = boost::lexical_cast<type>(impl_->getRawNode().text().as_string());                                   \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_pugixml
}  // namespace ariles2
