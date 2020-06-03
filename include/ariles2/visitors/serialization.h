/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

namespace ariles2
{
    namespace serialization
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters
        {
        public:
            bool sloppy_maps_;
            bool sloppy_pairs_;
            bool explicit_matrix_size_;


        public:
            Parameters()
            {
                sloppy_maps_ = false;
                sloppy_pairs_ = false;
                explicit_matrix_size_ = false;
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Features : public ariles2::Flags<unsigned int, serialization::Features>
        {
        public:
            enum Flags
            {
                RESET = 0,
                SLOPPY_MAPS_SUPPORTED = 1,

                DEFAULT = RESET
            };


        public:
            Features()
            {
                setDefaults();
            }

            Features(const unsigned int flags, const Action action_type = REPLACE)
            {
                initialize(flags, action_type);
            }


            void setDefaults()
            {
                flags_ = DEFAULT;
            }
        };


        template <class t_RawNode>
        class ARILES2_VISIBILITY_ATTRIBUTE Node
        {
        public:
            enum Type
            {
                UNDEFINED = 0,
                GENERIC = 1,
                ARRAY = 2,
                MATRIX = 3,
                VECTOR = 4
            };


        public:
            t_RawNode node_;
            std::size_t index_;
            std::size_t size_;
            Type type_;


        public:
            Node(t_RawNode node, const Type type = GENERIC) : node_(node)
            {
                type_ = type;
                index_ = 0;
                size_ = 0;
            }

            Node(const std::size_t index, const std::size_t size) : index_(index), size_(size)
            {
                type_ = ARRAY;
            }

            Node(t_RawNode node, const std::size_t index, const std::size_t size)
              : node_(node), index_(index), size_(size)
            {
                type_ = ARRAY;
            }

            bool isMatrix() const
            {
                return (MATRIX == type_);
            }

            bool isVector() const
            {
                return (VECTOR == type_);
            }

            bool isArray() const
            {
                return (ARRAY == type_);
            }

            bool isAllParsed() const
            {
                return (index_ == size_);
            }
        };


        template <class t_Parameters>
        class ARILES2_VISIBILITY_ATTRIBUTE Base : public visitor::Base<visitor::GenericVisitor, t_Parameters>
        {
        public:
            typedef t_Parameters Parameters;


        public:
            using visitor::Base<visitor::GenericVisitor, t_Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const t_Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }

            virtual const Features &getSerializationFeatures() const
            {
                static serialization::Features parameters;
                return (parameters);
            }
        };
    }  // namespace serialization
}  // namespace ariles2
