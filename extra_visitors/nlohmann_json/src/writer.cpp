/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include <ariles2/visitors/nlohmann_json.h>

#include "common.h"

#include <nlohmann/json.hpp>


namespace ariles2
{
    namespace ns_nlohmann_json
    {
        namespace impl
        {
            class Writer : public ariles2::ns_nlohmann_json::ImplBase<::nlohmann::ordered_json>,
                           public write::FileVisitorImplementation
            {
            public:
                template <class... t_Args>
                explicit Writer(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                    document_ = ::nlohmann::ordered_json::object();
                }
            };
        }  // namespace impl
    }  // namespace ns_nlohmann_json
}  // namespace ariles2


namespace ariles2
{
    namespace ns_nlohmann_json
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
            *impl_->output_stream_ << impl_->document_.dump(/*indent=*/4) << std::endl;
            impl_->output_stream_->flush();
        }


        void Writer::startMap(const Parameters &, const std::size_t /*num_entries*/)
        {
            impl_->getRawNode() = ::nlohmann::ordered_json::object();
        }

        void Writer::startMapEntry(const std::string &map_name)
        {
            ::nlohmann::ordered_json &obj = impl_->getRawNode();
            obj[map_name] = ::nlohmann::ordered_json();  // Create an empty value first

            // Find the newly added element and push it to the stack
            impl_->emplace(&obj.find(map_name).value());
        }

        void Writer::endMapEntry()
        {
            impl_->pop();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            CPPUT_TRACE_FUNCTION;
            impl_->getRawNode() = ::nlohmann::ordered_json::array();
            ::nlohmann::ordered_json &arr = impl_->getRawNode();

            // Add empty values
            for (std::size_t i = 0; i < size; ++i)
            {
                arr.push_back(::nlohmann::ordered_json());
            }
            impl_->emplace(0, size);
        }

        void Writer::startArrayElement()
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(
                    impl_->back().index_ < impl_->back().size_,
                    "Internal error: array has more elements than expected.");
        }

        void Writer::endArrayElement()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->shiftArray();
        }

        void Writer::endArray()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->pop();
        }


        /**
         * @brief Write a configuration entry
         *
         * @param[in] element data
         */
        void Writer::writeElement(const std::string &element, const Parameters &)
        {
            impl_->getRawNode() = element;
        }

        void Writer::writeElement(const bool &element, const Parameters &)
        {
            impl_->getRawNode() = element;
        }


        void Writer::writeElement(const float &element, const Parameters &param)
        {
            if (param.fallback_to_string_floats_)
            {
                impl_->getRawNode() = boost::lexical_cast<std::string>(element);
            }
            else
            {
                impl_->getRawNode() = static_cast<double>(element);
            }
        }


        void Writer::writeElement(const double &element, const Parameters &param)
        {
            if (param.fallback_to_string_floats_)
            {
                impl_->getRawNode() = boost::lexical_cast<std::string>(element);
            }
            else
            {
                impl_->getRawNode() = element;
            }
        }



#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        impl_->getRawNode() = static_cast<std::int64_t>(element);                                                      \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        impl_->getRawNode() = static_cast<std::uint64_t>(element);                                                     \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_nlohmann_json
}  // namespace ariles2
