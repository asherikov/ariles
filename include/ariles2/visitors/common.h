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
#include "../internal/helpers.h"

namespace ariles2
{
    namespace visitor
    {
        class Parameters
        {
        public:
            bool override_parameters_;

        public:
            Parameters(const bool override_parameters = true)
            {
                override_parameters_ = override_parameters;
            }
        };


        class Visitor
        {
        protected:
            Visitor(){};
            ~Visitor(){};
        };


        template <class t_Derived, class t_Parameters, class t_ReturnType = void>
        class ARILES2_VISIBILITY_ATTRIBUTE Base : public Visitor
        {
        public:
            using ReturnType = t_ReturnType;
            using Parameters = t_Parameters;

        public:
            const t_Parameters &getDefaultParameters() const
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
        ARILES2_TRACE_FUNCTION;                                                                                        \
        this->arilesVisit(visitor, param);                                                                             \
    }                                                                                                                  \
    using ariles2::Namespace::Base::arilesGetParameters;


    namespace entry
    {
        template <class t_Visitor>
        class ARILES2_VISIBILITY_ATTRIBUTE Base
        {
        public:
            virtual void arilesVirtualVisit(t_Visitor &, const typename t_Visitor::Parameters &) = 0;

            virtual const typename t_Visitor::Parameters &arilesGetParameters(const t_Visitor &visitor) const
            {
                ARILES2_TRACE_FUNCTION;
                return (visitor.getDefaultParameters());
            }
        };


        template <class t_Visitor>
        class ARILES2_VISIBILITY_ATTRIBUTE ConstBase
        {
        public:
            virtual void arilesVirtualVisit(t_Visitor &, const typename t_Visitor::Parameters &) const = 0;

            virtual const typename t_Visitor::Parameters &arilesGetParameters(const t_Visitor &visitor) const
            {
                ARILES2_TRACE_FUNCTION;
                return (visitor.getDefaultParameters());
            }
        };
    }  // namespace entry
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
    template <class t_Ariles, class t_Visitor, class t_Subtree>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const t_Subtree &subtree,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_ANY_OF(t_Subtree, std::string, std::vector<std::string>),
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        return (visitor.visit(ariles_class, subtree, param));
    }


    template <class t_Ariles, class t_Visitor>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const char *name,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        return (visitor.visit(ariles_class, name, param));
    }


    template <class t_Visitor, class t_Ariles>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        return (ariles2::apply(visitor, ariles_class, ariles_class.arilesDefaultID(), param));
    }


    template <class t_Visitor, class t_Ariles, class t_Subtree>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const t_Subtree &subtree,
            ARILES2_IS_ANY_OF(t_Subtree, std::string, std::vector<std::string>),
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        return (visitor.visit(ariles_class, subtree, visitor.getParameters(ariles_class)));
    }


    template <class t_Visitor, class t_Ariles>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const char *name,
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        return (visitor.visit(ariles_class, name, visitor.getParameters(ariles_class)));
    }


    template <class t_Visitor, class t_Ariles>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        return (ariles2::apply(visitor, ariles_class, ariles_class.arilesDefaultID()));
    }


    template <class t_Visitor, class t_Ariles>
    typename t_Visitor::ReturnType apply(
            t_Ariles &ariles_class,
            ARILES2_IS_ARILES_ENABLER(t_Ariles),
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor;
        return (ariles2::apply(visitor, ariles_class));
    }
    // -----



    // -----
    template <
            class t_Visitor,
            typename t_Arg,
            typename... t_Args,
            typename = std::enable_if_t<not std::is_base_of<ariles2::visitor::Visitor, t_Arg>::value>,
            typename = std::enable_if_t<not std::is_base_of<ariles2::Ariles, t_Arg>::value>,
            typename = std::enable_if_t<not std::is_same<std::string, std::decay_t<t_Arg>>::value>>
    typename t_Visitor::ReturnType apply(t_Arg &arg, t_Args &&...args)
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor(arg);
        return (ariles2::apply(visitor, std::forward<t_Args>(args)...));
    }


    template <
            class t_Visitor,
            typename t_Arg,
            typename... t_Args,
            typename = std::enable_if_t<not std::is_base_of<ariles2::visitor::Visitor, t_Arg>::value>,
            typename = std::enable_if_t<not std::is_same<char, std::decay_t<t_Arg>>::value>>
    typename t_Visitor::ReturnType apply(t_Arg *arg, t_Args &&...args)
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor(arg);
        return (ariles2::apply(visitor, std::forward<t_Args>(args)...));
    }


    template <class t_Visitor, typename... t_Args>
    typename t_Visitor::ReturnType apply(const std::string &arg, t_Args &&...args)
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor(arg);
        return (ariles2::apply(visitor, std::forward<t_Args>(args)...));
    }
    // -----


    // -----
    template <class t_Visitor, class t_Left, class t_Right>
    typename t_Visitor::ReturnType apply(
            t_Left &left,
            t_Right &right,
            ARILES2_IS_VISITOR_ENABLER(t_Visitor),
            ARILES2_IS_ARILES_ENABLER(t_Left))
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor;
        return (visitor.visit(left, right, left.arilesDefaultID(), visitor.getParameters(left)));
    }


    template <class t_Visitor, class t_Left, class t_Right>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Left &left,
            t_Right &right,
            const std::string &name,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        return (visitor.visit(left, right, name, param));
    }


    template <class t_Visitor, class t_Left, class t_Right>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Left &left,
            t_Right &right,
            ARILES2_IS_BASE_DISABLER(typename t_Visitor::Parameters, t_Right),
            ARILES2_IS_BASE_DISABLER(std::string, t_Right),
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        return (visitor.visit(left, right, left.arilesDefaultID(), visitor.getParameters(left)));
    }


    template <class t_Visitor, class t_Left, class t_Right>
    typename t_Visitor::ReturnType apply(
            t_Visitor &visitor,
            t_Left &left,
            t_Right &right,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_VISITOR_ENABLER(t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
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
