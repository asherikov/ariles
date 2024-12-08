/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <utility>
#include <vector>
#include <string_view>
#include "../internal/helpers.h"

namespace ariles2
{
    namespace visitor
    {
        class ARILES2_VISIBILITY_PUBLIC Parameters
        {
        public:
            bool override_parameters_;

        public:
            explicit Parameters(const bool override_parameters = true)
            {
                override_parameters_ = override_parameters;
            }
        };


        class ARILES2_VISIBILITY_PUBLIC Visitor
        {
        protected:
            Visitor() {};
            ~Visitor() {};
        };


        template <class t_Derived, class t_Parameters, class t_ReturnType = void>
        class ARILES2_VISIBILITY_PUBLIC Base : public Visitor
        {
        public:
            using ReturnType = t_ReturnType;
            using Parameters = t_Parameters;

        public:
            virtual const t_Parameters &getDefaultParameters() const
            {
                const static t_Parameters parameters(false);
                return parameters;
            }

            template <class t_Ariles>
            const t_Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*static_cast<const t_Derived *>(this)));
            }
        };
    }  // namespace visitor


#define ARILES2_BASE_METHODS(Namespace)                                                                                \
    using Namespace::Base::arilesVirtualVisit;                                                                         \
    using Namespace::Base::arilesGetParameters;

#define ARILES2_METHODS(Namespace, VisitorQualifier, MethodQualifier)                                                  \
    void arilesVirtualVisit(                                                                                           \
            VisitorQualifier ariles2::Namespace::Visitor &visitor,                                                     \
            const ariles2::Namespace::Visitor::Parameters &param) MethodQualifier override                             \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        this->arilesVisit(visitor, param);                                                                             \
    }                                                                                                                  \
    using ariles2::Namespace::Base::arilesGetParameters;


    namespace entry
    {
        template <class t_Visitor>
        class ARILES2_VISIBILITY_PUBLIC Base
        {
        public:
            virtual typename t_Visitor::ReturnType arilesVirtualVisit(
                    t_Visitor &,
                    const typename t_Visitor::Parameters &) = 0;

            virtual const typename t_Visitor::Parameters &arilesGetParameters(const t_Visitor &visitor) const
            {
                CPPUT_TRACE_FUNCTION;
                return (visitor.getDefaultParameters());
            }
        };


        template <class t_Visitor>
        class ARILES2_VISIBILITY_PUBLIC ConstBase
        {
        public:
            virtual typename t_Visitor::ReturnType arilesVirtualVisit(
                    t_Visitor &,
                    const typename t_Visitor::Parameters &) const = 0;

            virtual const typename t_Visitor::Parameters &arilesGetParameters(const t_Visitor &visitor) const
            {
                CPPUT_TRACE_FUNCTION;
                return (visitor.getDefaultParameters());
            }
        };
    }  // namespace entry

    namespace traits
    {
        template <class t_Subtree>
        using is_subtree = std::disjunction<
                std::is_same<std::string, std::decay_t<t_Subtree>>,       //
                std::is_same<char *, std::decay_t<t_Subtree>>,            //
                std::is_same<const char *, std::decay_t<t_Subtree>>,      //
                std::is_same<std::string_view, std::decay_t<t_Subtree>>,  //
                std::is_same<std::vector<std::string>, std::decay_t<t_Subtree>>>;

        template <class t_Subtree>
        using is_subtree_t = std::enable_if_t<is_subtree<t_Subtree>::value>;

        template <class t_Subtree>
        using is_not_subtree_t = std::enable_if_t<not is_subtree<t_Subtree>::value>;


        template <class t_Visitor>
        using is_visitor_t = std::enable_if_t<std::is_base_of_v<ariles2::visitor::Visitor, t_Visitor>>;

        template <class t_Visitor>
        using is_not_visitor_t = std::enable_if_t<not std::is_base_of_v<ariles2::visitor::Visitor, t_Visitor>>;

        template <class t_Ariles>
        using is_ariles_t = std::enable_if_t<std::is_base_of_v<ariles2::Ariles, t_Ariles>>;

        template <class t_Ariles>
        using is_not_ariles_t = std::enable_if_t<not std::is_base_of_v<ariles2::Ariles, t_Ariles>>;
    }  // namespace traits
}  // namespace ariles2

#ifndef ARILES2_DISABLE
#    ifndef ARILES2_ENABLED
#        define ARILES2_ENABLED
#    endif
#endif

#ifdef ARILES2_ENABLED

namespace ariles2
{
    // -----
    template <
            class t_Ariles,
            class t_Visitor,
            class t_Subtree,
            typename = traits::is_subtree_t<t_Subtree>,
            typename = traits::is_visitor_t<t_Visitor>>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            t_Subtree &&subtree,
            const typename t_Visitor::Parameters &param)
    {
        CPPUT_TRACE_FUNCTION;
        return (visitor.visit(ariles_class, std::forward<t_Subtree>(subtree), param));
    }


    template <class t_Visitor, class t_Ariles, typename = traits::is_visitor_t<t_Visitor>>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const typename t_Visitor::Parameters &param)
    {
        CPPUT_TRACE_FUNCTION;
        return (visitor.visit(ariles_class, ariles_class.arilesDefaultID(), param));
    }


    template <
            class t_Visitor,
            class t_Ariles,
            class t_Subtree,
            typename = traits::is_subtree_t<t_Subtree>,
            typename = traits::is_visitor_t<t_Visitor>>
    typename t_Visitor::ReturnType apply(t_Visitor &visitor, t_Ariles &ariles_class, t_Subtree &&subtree)
    {
        CPPUT_TRACE_FUNCTION;
        return (visitor.visit(ariles_class, std::forward<t_Subtree>(subtree), visitor.getParameters(ariles_class)));
    }


    template <class t_Visitor, class t_Ariles, typename = traits::is_visitor_t<t_Visitor>>
    typename t_Visitor::ReturnType apply(t_Visitor &visitor, t_Ariles &ariles_class)
    {
        CPPUT_TRACE_FUNCTION;
        return (ariles2::apply(visitor, ariles_class, ariles_class.arilesDefaultID()));
    }


    template <
            class t_Visitor,
            class t_Ariles,
            typename = traits::is_visitor_t<t_Visitor>,
            typename = traits::is_ariles_t<t_Ariles>>
    typename t_Visitor::ReturnType apply(t_Ariles &ariles_class)
    {
        CPPUT_TRACE_FUNCTION;
        t_Visitor visitor;
        return (ariles2::apply(visitor, ariles_class));
    }
    // -----



    // -----
    template <
            class t_Visitor,
            typename t_Arg,
            typename... t_Args,
            typename = traits::is_not_visitor_t<t_Arg>,
            typename = traits::is_not_ariles_t<t_Arg>>
    typename t_Visitor::ReturnType apply(t_Arg &&arg, t_Args &&...args)
    {
        CPPUT_TRACE_FUNCTION;
        t_Visitor visitor(std::forward<t_Arg>(arg));
        return (ariles2::apply(visitor, std::forward<t_Args>(args)...));
    }
    // -----


    // -----
    template <
            class t_Visitor,
            class t_Left,
            class t_Right,
            typename = traits::is_visitor_t<t_Visitor>,
            typename = traits::is_ariles_t<t_Left>,
            typename = traits::is_not_subtree_t<t_Right>>
    typename t_Visitor::ReturnType apply(t_Left &left, t_Right &right)
    {
        CPPUT_TRACE_FUNCTION;
        t_Visitor visitor;
        return (visitor.visit(left, right, left.arilesDefaultID(), visitor.getParameters(left)));
    }


    template <
            class t_Visitor,
            class t_Left,
            class t_Right,
            typename = traits::is_not_subtree_t<t_Right>,
            typename = traits::is_visitor_t<t_Visitor>>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Left &left,
            t_Right &right,
            const std::string &name,
            const typename t_Visitor::Parameters &param)
    {
        CPPUT_TRACE_FUNCTION;
        return (visitor.visit(left, right, name, param));
    }


    template <
            class t_Visitor,
            class t_Left,
            class t_Right,
            typename = traits::is_visitor_t<t_Visitor>,
            typename = traits::is_not_subtree_t<t_Right>,
            typename = std::enable_if_t<not std::is_base_of_v<typename t_Visitor::Parameters, t_Right>>>
    typename t_Visitor::ReturnType apply(t_Visitor &visitor, t_Left &left, t_Right &right)
    {
        CPPUT_TRACE_FUNCTION;
        return (visitor.visit(left, right, left.arilesDefaultID(), visitor.getParameters(left)));
    }


    template <
            class t_Visitor,
            class t_Left,
            class t_Right,
            typename = traits::is_not_subtree_t<t_Right>,
            typename = traits::is_visitor_t<t_Visitor>>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Left &left,
            t_Right &right,
            const typename t_Visitor::Parameters &param)
    {
        CPPUT_TRACE_FUNCTION;
        return (ariles2::apply(visitor, left, right, left.arilesDefaultID(), param));
    }
    // -----
}  // namespace ariles2

#else

namespace ariles2
{
    template <class t_Visitor, class... t_Args>
    void apply(t_Args &&.../*args*/)
    {
    }

    template <class... t_Args>
    void apply(t_Args &&.../*args*/)
    {
    }
}  // namespace ariles2

#endif
