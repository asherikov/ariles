/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "common.h"
#include <ariles2/visitors_impl/write.h>

namespace ariles2
{
    namespace ns_pugixml
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_PUBLIC Writer : public serialization::NodeStackBase<NodeWrapper>,
                                                     public write::FileVisitorImplementation
            {
            public:
                pugi::xml_document document_;


            public:
                template <class... t_Args>
                explicit Writer(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                    node_stack_.emplace_back(document_);
                }


                /**
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 */
                pugi::xml_node &getRawNode()
                {
                    return (back().node_);
                }
            };
        }  // namespace impl
    }  // namespace ns_pugixml
}  // namespace ariles2


namespace ariles2
{
    namespace ns_pugixml
    {
        Writer::Writer(const std::string &file_name)
        {
            makeImplPtr(file_name);
        }


        Writer::Writer(std::ostream &output_stream)
        {
            makeImplPtr(output_stream);
        }



        void Writer::flush()
        {
            impl_->document_.save(*impl_->output_stream_, "    ", pugi::format_indent);
            impl_->output_stream_->flush();
        }


        void Writer::startMapEntry(const std::string &map_name)
        {
            impl_->emplace(impl_->getRawNode().append_child(map_name.c_str()));
        }

        void Writer::endMapEntry()
        {
            impl_->pop();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            impl_->emplace(impl_->getRawNode(), 0, size);
        }

        void Writer::startArrayElement()
        {
            CPPUT_ASSERT(
                    impl_->back().index_ < impl_->back().size_,
                    "Internal error: array has more elements than expected.");
            impl_->emplace(impl_->getRawNode().append_child("item"));
        }

        void Writer::endArrayElement()
        {
            impl_->pop();
            impl_->shiftArray();
        }

        void Writer::endArray()
        {
            impl_->pop();
        }


        void Writer::startRoot(const std::string &name, const Parameters &)
        {
            CPPUT_TRACE_FUNCTION;
            if (name.empty())
            {
                startMapEntry("ariles");
            }
            else
            {
                startMapEntry(name);
            }
        }

        void Writer::endRoot(const std::string & /*name*/)
        {
            CPPUT_TRACE_FUNCTION;
            endMapEntry();
        }


        void Writer::writeElement(const std::string &element, const Parameters &)
        {
            impl_->getRawNode().text() = element.c_str();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        impl_->getRawNode().text() = (boost::lexical_cast<std::string>(element)).c_str();                              \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_pugixml
}  // namespace ariles2
