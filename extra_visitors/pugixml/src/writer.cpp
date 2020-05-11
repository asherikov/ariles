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
            class ARILES2_VISIBILITY_ATTRIBUTE Writer
            {
            public:
                pugi::xml_document document_;

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
                    node_stack_.push_back(document_);
                }


                explicit Writer(std::ostream &output_stream)
                {
                    output_stream_ = &output_stream;
                    node_stack_.push_back(document_);
                }


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
        Writer::Writer(const std::string &file_name)
        {
            impl_ = ImplPtr(new Impl(file_name));
        }


        Writer::Writer(std::ostream &output_stream)
        {
            impl_ = ImplPtr(new Impl(output_stream));
        }



        void Writer::flush()
        {
            impl_->document_.save(*impl_->output_stream_, "    ", pugi::format_indent);
            impl_->output_stream_->flush();
        }


        void Writer::descend(const std::string &map_name)
        {
            impl_->node_stack_.push_back(impl_->getRawNode().append_child(map_name.c_str()));
        }

        void Writer::ascend()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            impl_->node_stack_.push_back(NodeWrapper(impl_->getRawNode(), 0, size));
            if (size > 0)
            {
                impl_->node_stack_.push_back(impl_->getRawNode().append_child("item"));
            }
        }

        void Writer::shiftArray()
        {
            impl_->node_stack_.pop_back();
            ARILES2_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: expected array.");
            ARILES2_ASSERT(
                    impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_,
                    "Internal error: array has more elements than expected.");
            ++impl_->node_stack_.back().index_;
            if (impl_->node_stack_.back().index_ < impl_->node_stack_.back().size_)
            {
                impl_->node_stack_.push_back(impl_->getRawNode().append_child("item"));
            }
        }

        void Writer::endArray()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startRoot(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == name.empty())
            {
                descend("ariles");
            }
            else
            {
                descend(name);
            }
        }

        void Writer::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            ascend();
        }


        void Writer::writeElement(const std::string &element)
        {
            impl_->getRawNode().text() = element.c_str();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element)                                                                     \
    {                                                                                                                  \
        impl_->getRawNode().text() = (boost::lexical_cast<std::string>(element)).c_str();                              \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_pugixml
}  // namespace ariles2
