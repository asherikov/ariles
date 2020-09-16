/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

/**
@defgroup serialization Serialization

@brief Serialization.
*/

namespace ariles2
{
    /// @ingroup serialization
    namespace serialization
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public visitor::Parameters
        {
        public:
            bool sloppy_maps_;
            bool sloppy_pairs_;
            bool explicit_matrix_size_;
            bool fallback_to_string_floats_;


        public:
            Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
                sloppy_maps_ = false;
                sloppy_pairs_ = false;
                explicit_matrix_size_ = false;
                fallback_to_string_floats_ = true;
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
                VECTOR = 4,
                ITERATED_MAP = 5
            };


        public:
            t_RawNode node_;
            std::size_t index_;
            std::size_t size_;
            Type type_;


        public:
            Node(const Type type = GENERIC)
            {
                ARILES2_TRACE_FUNCTION
                type_ = type;
            }

            Node(t_RawNode node, const Type type = GENERIC) : node_(node)
            {
                ARILES2_TRACE_FUNCTION
                type_ = type;
                index_ = 0;
                size_ = 0;
            }

            Node(const std::size_t index, const std::size_t size) : index_(index), size_(size)
            {
                ARILES2_TRACE_FUNCTION
                type_ = ARRAY;
            }

            Node(t_RawNode node, const std::size_t index, const std::size_t size)
              : node_(node), index_(index), size_(size)
            {
                ARILES2_TRACE_FUNCTION
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


        template <class t_Visitor, class t_Implementation>
        class ARILES2_VISIBILITY_ATTRIBUTE PIMPLVisitor : public t_Visitor
        {
        protected:
            typedef t_Implementation Impl;
            typedef ARILES2_SHARED_PTR<t_Implementation> ImplPtr;

        protected:
            ImplPtr impl_;

        private:
            PIMPLVisitor(const PIMPLVisitor &);
            PIMPLVisitor &operator=(const PIMPLVisitor &);

        protected:
            PIMPLVisitor(){};
            ~PIMPLVisitor(){};
        };


        template <class t_Derived, class t_Parameters>
        class ARILES2_VISIBILITY_ATTRIBUTE Base : public visitor::Base<visitor::GenericVisitor, t_Parameters>
        {
        public:
            typedef t_Parameters Parameters;


        public:
            using visitor::Base<visitor::GenericVisitor, t_Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const t_Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*static_cast<const t_Derived *>(this)));
            }
        };
    }  // namespace serialization
}  // namespace ariles2
