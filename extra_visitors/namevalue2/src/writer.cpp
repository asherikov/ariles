/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/namevalue2.h>
#include <ariles2/visitors_impl/serialization.h>

#include <boost/lexical_cast.hpp>

namespace ariles2
{
    namespace ns_namevalue2
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_PUBLIC Writer : public serialization::NodeStackBase<serialization::NodeString>
            {
            public:
                bool initialize_names_;
                std::size_t index_;
                std::shared_ptr<NameValueContainer> name_value_pairs_;

                const std::string separator_ = ".";
                const std::string bracket_left_ = "{";
                const std::string bracket_right_ = "}";

            public:
                Writer(const std::shared_ptr<NameValueContainer> &container, const std::size_t reserve)
                {
                    name_value_pairs_ = container;
                    name_value_pairs_->reserve(reserve);
                    initialize_names_ = true;
                    index_ = 0;
                }


                void flush()
                {
                    if (initialize_names_)
                    {
                        // drop trailing leftovers
                        name_value_pairs_->resize(index_);
                        initialize_names_ = false;
                    }
                    index_ = 0;
                }

                void startRoot(const bool persistent_structure)
                {
                    if (not persistent_structure or 0 == name_value_pairs_->size())
                    {
                        initialize_names_ = true;
                    }
                }

                void startMap(const std::size_t num_entries)  // NOLINT
                {
                    if (initialize_names_)
                    {
                        name_value_pairs_->reserve(index_ + num_entries);
                    }
                }

                void startMapEntry(const std::string &map_name)
                {
                    if (initialize_names_)
                    {
                        if (empty())
                        {
                            emplace(map_name);
                        }
                        else
                        {
                            if (back().isArray())
                            {
                                concatWithNodeAndEmplace(
                                        bracket_left_,
                                        boost::lexical_cast<std::string>(back().index_),
                                        bracket_right_,
                                        separator_,
                                        map_name);
                            }
                            else
                            {
                                concatWithNodeAndEmplace(separator_, map_name);
                            }
                        }
                    }
                }

                void endMapEntry()
                {
                    if (initialize_names_)
                    {
                        pop();
                    }
                }

                void startArray(const std::size_t size)
                {
                    if (initialize_names_)
                    {
                        name_value_pairs_->reserve(index_ + size);
                        if (back().isArray())
                        {
                            emplace(concatWithNode(std::string("_"), boost::lexical_cast<std::string>(back().index_)),
                                    0,
                                    size);
                        }
                        else
                        {
                            emplace(back().node_, 0, size);
                        }
                    }
                }

                template <class t_Element>
                void writeElement(const t_Element &element)
                {
                    if (index_ == name_value_pairs_->size())
                    {
                        name_value_pairs_->resize(index_ + 1);
                    }
                    if (initialize_names_)
                    {
                        name_value_pairs_->name(index_) = back().node_;
                        if (back().isArray())
                        {
                            name_value_pairs_->name(index_) += "_";
                            name_value_pairs_->name(index_) += boost::lexical_cast<std::string>(back().index_);
                        }
                    }
                    name_value_pairs_->value(index_) = element;
                    ++index_;
                }
            };
        }  // namespace impl
    }  // namespace ns_namevalue2
}  // namespace ariles2


namespace ariles2
{
    namespace ns_namevalue2
    {
        Writer::Writer(const std::shared_ptr<NameValueContainer> &container, const std::size_t reserve)
        {
            makeImplPtr(container, reserve);
        }

        void Writer::startRoot(const std::string &name, const Writer::Parameters &param)
        {
            CPPUT_TRACE_FUNCTION;
            impl_->startRoot(param.persistent_structure_);

            if (not name.empty())
            {
                startMapEntry(name);
            }
        }

        void Writer::flush()
        {
            impl_->flush();
        }


        void Writer::startMap(const Writer::Parameters &, const std::size_t num_entries)
        {
            impl_->startMap(num_entries);
        }

        void Writer::startMapEntry(const std::string &map_name)
        {
            impl_->startMapEntry(map_name);
        }

        void Writer::endMapEntry()
        {
            impl_->endMapEntry();
        }

        void Writer::endMap()
        {
        }


        bool Writer::startIteratedMap(const std::size_t /*num_entries*/, const Writer::Parameters &)
        {
            return (false);
        }

        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            impl_->startArray(size);
        }

        void Writer::endArrayElement()
        {
            if (impl_->initialize_names_)
            {
                impl_->shiftArray();
            }
        }

        void Writer::endArray()
        {
            if (impl_->initialize_names_)
            {
                impl_->pop();
            }
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Writer::Parameters &)                                         \
    {                                                                                                                  \
        impl_->writeElement(element);                                                                                  \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


        void Writer::writeElement(const std::string &element, const Writer::Parameters &parameters)
        {
            writeElement(element.size(), parameters);
        }

        const Writer::Parameters &Writer::getDefaultParameters() const
        {
            static Writer::Parameters parameters(/*override_parameters=*/true);

            parameters.sloppy_maps_ = true;
            parameters.sloppy_pairs_ = true;
            parameters.explicit_matrix_size_ = false;
            parameters.fallback_to_string_floats_ = false;
            parameters.flat_matrices_ = false;
            parameters.allow_missing_entries_ = true;

            parameters.persistent_structure_ = false;

            return parameters;
        }
    }  // namespace ns_namevalue2
}  // namespace ariles2
