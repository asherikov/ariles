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
    namespace visitor
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
        {
        protected:
            Visitor(){};
            ~Visitor(){};
        };


        class ARILES2_VISIBILITY_ATTRIBUTE GenericVisitor : public Visitor
        {
        protected:
            GenericVisitor(){};
            ~GenericVisitor(){};
        };


        template <class t_Visitor, class t_Parameters>
        class ARILES2_VISIBILITY_ATTRIBUTE Base : public t_Visitor
        {
        protected:
            Base(){};
            ~Base(){};

            const t_Parameters &getDefaultParameters() const
            {
                const static t_Parameters parameters;
                return parameters;
            }
        };
    }  // namespace visitor


#define ARILES2_BASE_METHODS(Namespace)                                                                                \
    using Namespace::Base::arilesVirtualVisit;                                                                         \
    using Namespace::Base::arilesGetParameters;

#define ARILES2_METHODS(Namespace, VisitorQualifier, MethodQualifier)                                                  \
    virtual void arilesVirtualVisit(                                                                                   \
            VisitorQualifier ariles2::Namespace::Visitor &visitor,                                                     \
            const ariles2::Namespace::Visitor::Parameters &param) MethodQualifier                                      \
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
    template <class t_Ariles, class t_Visitor>
    void apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const std::string &name,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        visitor.start(ariles_class, name, param);
    }


    template <class t_Ariles, class t_Visitor>
    void apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const char *name,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        visitor.start(ariles_class, name, param);
    }


    template <class t_Visitor, class t_Ariles>
    void apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        ariles2::apply(visitor, ariles_class, ariles_class.arilesDefaultID(), param);
    }


    template <class t_Visitor, class t_Ariles>
    void apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const std::string &name,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        ariles2::apply(visitor, ariles_class, name, visitor.getParameters(ariles_class));
    }


    template <class t_Visitor, class t_Ariles>
    void apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            const char *name,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        ariles2::apply(visitor, ariles_class, name, visitor.getParameters(ariles_class));
    }


    template <class t_Visitor, class t_Ariles>
    void apply(
            t_Visitor &visitor,
            t_Ariles &ariles_class,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        ariles2::apply(visitor, ariles_class, ariles_class.arilesDefaultID());
    }


    template <class t_Visitor, class t_Ariles>
    void apply(t_Ariles &ariles_class, ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor;
        ariles2::apply(visitor, ariles_class);
    }
    // -----



    // -----
    template <class t_Visitor, class t_Ariles, class t_Arg>
    void apply(
            t_Arg &arg,
            t_Ariles &ariles_class,
            ARILES2_IS_BASE_DISABLER(ariles2::visitor::Visitor, t_Arg),
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor(arg);
        ariles2::apply(visitor, ariles_class);
    }


    template <class t_Visitor, class t_Ariles, class t_Arg>
    void apply(
            t_Arg &arg,
            t_Ariles &ariles_class,
            const char *name,
            ARILES2_IS_BASE_DISABLER(ariles2::visitor::Visitor, t_Arg),
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor(arg);
        ariles2::apply(visitor, ariles_class, name);
    }


    template <class t_Visitor, class t_Ariles, class t_Arg>
    void apply(
            const t_Arg &arg,
            t_Ariles &ariles_class,
            ARILES2_IS_BASE_DISABLER(ariles2::visitor::Visitor, t_Arg),
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor(arg);
        ariles2::apply(visitor, ariles_class);
    }


    template <class t_Visitor, class t_Ariles, class t_Arg>
    void apply(
            t_Arg &arg,
            t_Ariles &ariles_class,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_BASE_DISABLER(ariles2::visitor::Visitor, t_Arg),
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor(arg);
        ariles2::apply(visitor, ariles_class, param);
    }


    template <class t_Visitor, class t_Ariles>
    void apply(
            const std::string &arg,
            t_Ariles &ariles_class,
            const unsigned int flags,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        t_Visitor visitor(arg);
        ariles2::apply(visitor, ariles_class, typename t_Visitor::Parameters(flags));
    }
    // -----


    // -----
    template <class t_Visitor, class t_Left, class t_Right>
    void apply(
            t_Visitor &visitor,
            t_Left &left,
            t_Right &right,
            const std::string &name,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        try
        {
            visitor.start(left, right, name, param);
        }
        catch (std::exception &e)
        {
            if (true == param.throw_on_error_)
            {
                ARILES2_THROW(std::string("Comparison failed: ") + e.what());
            }
            visitor.equal_ = false;
        }
    }


    template <class t_Visitor, class t_Left, class t_Right>
    void apply(
            t_Visitor &visitor,
            t_Left &left,
            t_Right &right,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        ariles2::apply(visitor, left, right, left.arilesDefaultID(), visitor.getParameters(left));
    }


    template <class t_Visitor, class t_Left, class t_Right>
    void apply(
            t_Visitor &visitor,
            t_Left &left,
            t_Right &right,
            const typename t_Visitor::Parameters &param,
            ARILES2_IS_BASE_ENABLER(ariles2::visitor::Visitor, t_Visitor))
    {
        ARILES2_TRACE_FUNCTION;
        ariles2::apply(visitor, left, right, left.arilesDefaultID(), param);
    }
    // -----
}  // namespace ariles2

#else

namespace ariles2
{
    // -----
    template <class t_One, class t_Two, class t_Three, class t_Four, class t_Five>
    void apply(t_One, t_Two, t_Three, t_Four, t_Five)
    {
    }

    template <class t_One, class t_Two, class t_Three, class t_Four>
    void apply(t_One, t_Two, t_Three, t_Four)
    {
    }

    template <class t_One, class t_Two, class t_Three>
    void apply(t_One, t_Two, t_Three)
    {
    }

    template <class t_One, class t_Two>
    void apply(t_One, t_Two)
    {
    }

    template <class t_One, class t_Two>
    void apply(t_Two)
    {
    }
    // -----
}  // namespace ariles2

#endif
