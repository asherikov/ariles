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
            bool sloppy_maps_;                /// Treat key values in maps as entry names if they are strings
            bool sloppy_pairs_;               /// Treat first entry in an std::pair as entry name if it is a string
            bool explicit_matrix_size_;       /// Specify matrix size even if it is known to be constant
            bool fallback_to_string_floats_;  /// Allow saving floats as strings if necessary
            bool flat_matrices_;              /// Save matrix as a single vector
            bool allow_missing_entries_;      /// Do not treat missing entries as errors
            bool persistent_structure_;       /// Hint: expect Ariles classes with constant number of entries


        public:
            Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
                sloppy_maps_ = false;
                sloppy_pairs_ = false;
                explicit_matrix_size_ = false;
                fallback_to_string_floats_ = true;
                flat_matrices_ = true;
                allow_missing_entries_ = false;
                persistent_structure_ = false;
            }
        };


        template <class t_RawNode>
        class ARILES2_VISIBILITY_ATTRIBUTE Node
        {
        public:
            enum class Type
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
            Node(const Type type = Type::GENERIC)
            {
                CPPUT_TRACE_FUNCTION
                type_ = type;
            }

            Node(t_RawNode node, const Type type = Type::GENERIC) : node_(node)
            {
                CPPUT_TRACE_FUNCTION
                type_ = type;
                index_ = 0;
                size_ = 0;
            }

            Node(const std::size_t index, const std::size_t size) : index_(index), size_(size)
            {
                CPPUT_TRACE_FUNCTION
                type_ = Type::ARRAY;  // NOLINT
            }                         // NOLINT

            Node(t_RawNode node, const std::size_t index, const std::size_t size)
              : node_(node), index_(index), size_(size)
            {
                CPPUT_TRACE_FUNCTION
                type_ = Type::ARRAY;
            }

            bool isMatrix() const
            {
                return (Type::MATRIX == type_);
            }

            bool isVector() const
            {
                return (Type::VECTOR == type_);
            }

            bool isArray() const
            {
                return (Type::ARRAY == type_);
            }

            bool isCompleted() const
            {
                return (index_ >= size_);
            }
        };


        template <class t_Node>
        class NodeStackBase
        {
        public:
            std::vector<t_Node> node_stack_;

        public:
            [[nodiscard]] t_Node &back()
            {
                return (node_stack_.back());
            }

            [[nodiscard]] const t_Node &back() const
            {
                return (node_stack_.back());
            }

            void clear()
            {
                node_stack_.clear();
            }

            template <class... t_Args>
            void emplace(t_Args &&...args)
            {
                node_stack_.emplace_back(std::forward<t_Args>(args)...);
            }

            void pop()
            {
                node_stack_.pop_back();
            }

            void shiftArray()
            {
                CPPUT_ASSERT(back().isArray(), "Internal error: expected array.");
                ++back().index_;
            }

            bool empty() const
            {
                return (node_stack_.empty());
            }


            template <typename... t_String>
            std::string concatWithNode(t_String &&...strings) const
            {
                return (cpput::concat::simple(back().node_, std::forward<t_String>(strings)...));
            }

            template <typename... t_String>
            void concatWithNodeAndEmplace(t_String &&...strings)
            {
                emplace(concatWithNode(std::forward<t_String>(strings)...));
            }
        };


        template <class t_Visitor, class t_Implementation>
        class ARILES2_VISIBILITY_ATTRIBUTE PIMPLVisitor : public t_Visitor
        {
        protected:
            using Impl = t_Implementation;
            using ImplPtr = std::shared_ptr<t_Implementation>;

        protected:
            ImplPtr impl_;

        private:
            PIMPLVisitor(const PIMPLVisitor &);
            PIMPLVisitor &operator=(const PIMPLVisitor &);

        protected:
            PIMPLVisitor(){};
            ~PIMPLVisitor(){};

            template <class... t_Args>
            void makeImplPtr(t_Args &&...args)
            {
                impl_ = std::make_shared<Impl>(std::forward<t_Args>(args)...);
            }
        };


        template <class t_Derived, class t_Parameters>
        using Base = visitor::Base<t_Derived, t_Parameters>;
    }  // namespace serialization
}  // namespace ariles2
