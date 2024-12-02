/**
    @file
    @author Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <string>
#include <vector>
#include <utility>
#include <boost/lexical_cast.hpp>
#include <ariles2/visitors_impl/serialization.h>


namespace ariles2
{
    namespace ns_namevalue
    {
        using NameValuePair = std::pair<std::string, double>;


        template <class t_NameValuePair>
        class ARILES2_VISIBILITY_ATTRIBUTE NameValuePairHandler
        {
        };


        template <>
        class ARILES2_VISIBILITY_ATTRIBUTE NameValuePairHandler<NameValuePair>
        {
        public:
            static inline std::string &name(NameValuePair &pair)  // cppcheck-suppress constParameter
            {
                return (pair.first);
            }

            static inline double &value(NameValuePair &pair)  // cppcheck-suppress constParameter
            {
                return (pair.second);
            }
        };


        /**
         * @brief Configuration writer class
         */
        template <class t_NameValuePair>
        class ARILES2_VISIBILITY_ATTRIBUTE GenericWriter
          : public ariles2::write::Visitor,
            public serialization::NodeStackBase<serialization::Node<std::string>>
        {
        protected:
            std::size_t reserve_;

            std::vector<t_NameValuePair> buffer_name_value_pairs_;

            bool initialize_structure_;

            const std::string separator_ = ".";
            const std::string bracket_left_ = "{";
            const std::string bracket_right_ = "}";


        public:
            std::vector<t_NameValuePair> *name_value_pairs_;
            std::size_t index_;


        protected:
            void expand()
            {
                if (index_ == name_value_pairs_->size())
                {
                    name_value_pairs_->resize(name_value_pairs_->size() + 1);
                }
            }

            void expandReserve(const std::size_t size)
            {
                reserve_ += size;
                name_value_pairs_->reserve(reserve_);
            }

            void clear()
            {
                name_value_pairs_->clear();
            }


        public:
            explicit GenericWriter(const std::size_t reserve = 0)
            {
                name_value_pairs_ = &buffer_name_value_pairs_;

                if (reserve > 0)
                {
                    expandReserve(reserve);
                }
                reset();
            }


            explicit GenericWriter(std::vector<t_NameValuePair> *name_value_pairs, const std::size_t reserve = 0)
            {
                name_value_pairs_ = name_value_pairs;

                if (reserve > 0)
                {
                    expandReserve(reserve);
                }
                reset();
            }


            void flush()
            {
            }


            void reset(const bool initialize_structure = true)
            {
                if (initialize_structure)
                {
                    clear();
                }
                initialize_structure_ = initialize_structure;
                index_ = 0;
                reserve_ = 0;
            }


            virtual void startMap(const Parameters &, const std::size_t num_entries)
            {
                if (initialize_structure_)
                {
                    expandReserve(num_entries);
                }
            }

            virtual void startMapEntry(const std::string &map_name)
            {
                if (initialize_structure_)
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

            virtual void endMapEntry()
            {
                if (initialize_structure_)
                {
                    pop();
                }
            }

            virtual void endMap()
            {
            }


            virtual bool startIteratedMap(const std::size_t /*num_entries*/, const Parameters &)
            {
                return (false);
            }

            virtual void startArray(const std::size_t size, const bool /*compact*/ = false)
            {
                if (initialize_structure_)
                {
                    expandReserve(size);
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

            virtual void endArrayElement()
            {
                if (initialize_structure_)
                {
                    shiftArray();
                }
            }

            virtual void endArray()
            {
                if (initialize_structure_)
                {
                    pop();
                }
            }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void writeElement(const type &element, const Parameters &)                                                         \
    {                                                                                                                  \
        expand();                                                                                                      \
        if (initialize_structure_)                                                                                     \
        {                                                                                                              \
            NameValuePairHandler<t_NameValuePair>::name((*name_value_pairs_)[index_]) = back().node_;                  \
            if (back().isArray())                                                                                      \
            {                                                                                                          \
                NameValuePairHandler<t_NameValuePair>::name((*name_value_pairs_)[index_]) += "_";                      \
                NameValuePairHandler<t_NameValuePair>::name((*name_value_pairs_)[index_]) +=                           \
                        boost::lexical_cast<std::string>(back().index_);                                               \
            }                                                                                                          \
        }                                                                                                              \
        NameValuePairHandler<t_NameValuePair>::value((*name_value_pairs_)[index_]) = element;                          \
        ++index_;                                                                                                      \
    }

            CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


            void writeElement(const std::string & /*element*/, const Parameters &)
            {
            }
        };


        using Writer = GenericWriter<NameValuePair>;
    }  // namespace ns_namevalue
}  // namespace ariles2
