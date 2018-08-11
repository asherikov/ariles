/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    template <class t_RawNode>
        class ARILES_VISIBILITY_ATTRIBUTE Node
    {
        public:
            enum Type
            {
                UNDEFINED = 0,
                GENERIC = 1,
                ARRAY = 2,
                MATRIX = 3
            };


        public:
            t_RawNode           node_;
            std::size_t         index_;
            std::size_t         size_;
            Type                type_;
            bool                compact_;


        public:
            Node(t_RawNode node, const Type type = GENERIC, const bool compact = false) : node_(node)
            {
                type_ = type;
                index_ = 0;
                size_ = 0;
                compact_ = compact;
            }

            Node(const std::size_t index, const std::size_t size, const bool compact = false)
                : index_(index),
                  size_(size)
            {
                type_ = ARRAY;
                compact_ = compact;
            }

            Node(t_RawNode node, const std::size_t index, const std::size_t size, const bool compact = false)
                : node_(node),
                  index_(index),
                  size_(size)
            {
                type_ = ARRAY;
                compact_ = compact;
            }

            bool isCompact() const
            {
                return(compact_);
            }

            bool isMatrix() const
            {
                return(MATRIX == type_);
            }

            bool isArray() const
            {
                return(ARRAY == type_);
            }

            bool isAllParsed() const
            {
                return(index_ == size_);
            }
    };
}
