/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles2
{
    namespace serialization
    {
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
            std::size_t index_;
            std::size_t size_;
            Type type_;


        public:
            explicit Node(const Type type = Type::GENERIC)
            {
                CPPUT_TRACE_FUNCTION;
                type_ = type;
                index_ = 0;
                size_ = 0;
            }

            Node(const std::size_t index, const std::size_t size) : index_(index), size_(size)
            {
                CPPUT_TRACE_FUNCTION;
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


        template <class t_Node, class t_NodeArg = t_Node>
        class NodeTemplate : public serialization::Node
        {
        public:
            t_Node node_;

        public:
            template <class... t_Args>
            NodeTemplate(t_NodeArg node, t_Args &&...args) : serialization::Node(std::forward<t_Args>(args)...)
            {
                node_ = node;
            }
        };

        using NodeString = NodeTemplate<std::string, const std::string &>;


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
    }  // namespace serialization
}  // namespace ariles2
