/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <boost/math/special_functions.hpp>
#include <ariles2/visitors/nlohmann_json.h>
#include "common.h"


namespace ariles2
{
    namespace ns_nlohmann_json
    {
        namespace impl
        {
            class Reader : public ariles2::ns_nlohmann_json::ImplBase<const ::nlohmann::ordered_json>,
                           public read::FileVisitorImplementation
            {
            public:
                std::vector<::nlohmann::ordered_json::const_iterator> iterator_stack_;

            public:
                Reader() = default;

                template <class... t_Args>
                explicit Reader(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                    initialize();
                }


                void initialize()
                {
                    document_ = ::nlohmann::ordered_json::parse(*input_stream_);
                }
            };
        }  // namespace impl
    }  // namespace ns_nlohmann_json
}  // namespace ariles2



namespace ariles2
{
    namespace ns_nlohmann_json
    {
        Reader::Reader(const std::string &file_name)
        {
            CPPUT_TRACE_FUNCTION;
            makeImplPtr(file_name);
        }


        Reader::Reader(std::istream &input_stream)
        {
            CPPUT_TRACE_FUNCTION;
            makeImplPtr(input_stream);
        }


        void Reader::constructFromString(const char *input_string)
        {
            CPPUT_TRACE_FUNCTION;
            makeImplPtr();
            impl_->document_ = ::nlohmann::ordered_json::parse(input_string);
        }


        void Reader::startMap(const SizeLimitEnforcementType limit_type, const std::size_t min, const std::size_t max)
        {
            CPPUT_TRACE_FUNCTION;
            checkSize(limit_type, impl_->getRawNode().size(), min, max);
        }

        bool Reader::startMapEntry(const std::string &child_name)
        {
            const ::nlohmann::ordered_json &obj = impl_->getRawNode();
            const ::nlohmann::ordered_json::const_iterator child = obj.find(child_name);

            if (obj.end() == child)
            {
                return (false);
            }
            impl_->emplace(&child.value());
            return (true);
        }

        void Reader::endMapEntry()
        {
            impl_->pop();
        }


        bool Reader::startIteratedMap(
                const SizeLimitEnforcementType limit_type,
                const std::size_t min,
                const std::size_t max)
        {
            CPPUT_TRACE_FUNCTION;
            const ::nlohmann::ordered_json &obj = impl_->getRawNode();
            checkSize(limit_type, obj.size(), min, max);


            if (impl_->getRawNode().is_object())
            {
                impl_->iterator_stack_.push_back(obj.begin());
                return (true);
            }
            return (false);
        }

        bool Reader::startIteratedMapElement(std::string &entry_name)
        {
            const auto &current_iter = impl_->iterator_stack_.back();
            if (current_iter != impl_->getRawNode().end())
            {
                impl_->emplace(&current_iter.value());
                entry_name = current_iter.key();
                return (true);
            }
            return (false);
        }

        void Reader::endIteratedMapElement()
        {
            ++impl_->iterator_stack_.back();
            impl_->pop();
        }

        void Reader::endIteratedMap()
        {
            CPPUT_ASSERT(impl_->iterator_stack_.back() == impl_->getRawNode().end(), "End of iterated map has not been reached.");
            impl_->iterator_stack_.pop_back();
        }


        std::size_t Reader::startArray()
        {
            const std::size_t size = impl_->getRawNode().size();
            impl_->emplace(0, size);  // index=0, size=size
            return (size);
        }


        void Reader::startArrayElement()
        {
            CPPUT_ASSERT(
                    impl_->back().index_ < impl_->back().size_,
                    "Internal error: array has more elements than expected.");
        }


        void Reader::endArrayElement()
        {
            impl_->shiftArray();
        }


        void Reader::endArray()
        {
            impl_->pop();
        }


        void Reader::readElement(std::string &element)
        {
            element = impl_->getRawNode().get<std::string>();
        }


        void Reader::readElement(bool &element)
        {
            element = impl_->getRawNode().get<bool>();
        }


        void Reader::readElement(float &element)
        {
            float tmp_value = 0.0;
            if (impl_->getRawNode().is_string())
            {
                tmp_value = boost::lexical_cast<float>(impl_->getRawNode().get<std::string>());
                if (boost::math::isnan(tmp_value))
                {
                    element = std::numeric_limits<float>::signaling_NaN();
                    return;
                }
                if (boost::math::isinf(tmp_value))
                {
                    element = tmp_value;
                    return;
                }
            }
            else
            {
                tmp_value = static_cast<float>(impl_->getRawNode().get<double>());
            }
            CPPUT_ASSERT(
                    tmp_value <= std::numeric_limits<float>::max() && tmp_value >= -std::numeric_limits<float>::max(),
                    "Value is out of range.");
            element = tmp_value;
        }


        void Reader::readElement(double &element)
        {
            double tmp_value = 0.0;
            if (impl_->getRawNode().is_string())
            {
                tmp_value = boost::lexical_cast<double>(impl_->getRawNode().get<std::string>());
                if (boost::math::isnan(tmp_value))
                {
                    element = std::numeric_limits<double>::signaling_NaN();
                    return;
                }
                if (boost::math::isinf(tmp_value))
                {
                    element = tmp_value;
                    return;
                }
            }
            else
            {
                tmp_value = impl_->getRawNode().get<double>();
            }
            CPPUT_ASSERT(
                    tmp_value <= std::numeric_limits<double>::max() && tmp_value >= -std::numeric_limits<double>::max(),
                    "Value is out of range.");
            element = tmp_value;
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        const int64_t tmp_value = impl_->getRawNode().get<int64_t>();                                                  \
        CPPUT_ASSERT(                                                                                                  \
                tmp_value <= std::numeric_limits<type>::max() && tmp_value >= std::numeric_limits<type>::min(),        \
                "Value is out of range.");                                                                             \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_SIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Reader::readElement(type &element)                                                                            \
    {                                                                                                                  \
        const uint64_t tmp_value = impl_->getRawNode().get<uint64_t>();                                                \
        CPPUT_ASSERT(tmp_value <= std::numeric_limits<type>::max(), "Value is too large.");                            \
        element = static_cast<type>(tmp_value);                                                                        \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_UNSIGNED_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_nlohmann_json
}  // namespace ariles2
