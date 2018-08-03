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
            t_RawNode           node_;
            std::size_t         index_;
            std::size_t         size_;
            bool                is_array_;

        public:
            Node(t_RawNode node) : node_(node)
            {
                is_array_ = false;
                index_ = 0;
                size_ = 0;
            }

            Node(const std::size_t index, const std::size_t size)
                : index_(index),
                  size_(size)
            {
                is_array_ = true;
            }

            Node(t_RawNode node, const std::size_t index, const std::size_t size)
                : node_(node),
                  index_(index),
                  size_(size)
            {
                is_array_ = true;
            }

            bool isArray() const
            {
                return(true == is_array_);
            }

            bool isAllParsed() const
            {
                return(index_ == size_);
            }
    };
}
