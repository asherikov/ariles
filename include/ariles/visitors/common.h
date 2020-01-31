/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    namespace visitor
    {
        struct ARILES_VISIBILITY_ATTRIBUTE Visitor
        {
        };


        #define ARILES_VISITOR_BASE(ClassName, Qualifier) \
            template<class t_Derived, class t_Visitor> \
                class ARILES_VISIBILITY_ATTRIBUTE ClassName \
            { \
                public: \
                    virtual void arilesVirtualVisit( \
                            t_Visitor &, \
                            const typename t_Visitor::Parameters &) Qualifier = 0; \
                    \
                    virtual void ariles( \
                            t_Visitor &visitor, \
                            const std::string & name, \
                            const typename t_Visitor::Parameters &param) Qualifier = 0; \
                    \
                    void ariles(t_Visitor &visitor, \
                                const typename t_Visitor::Parameters &param) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        ariles(visitor, static_cast<t_Derived*>(this)->arilesDefaultID(), param); \
                    } \
                    \
                    void ariles(t_Visitor &visitor) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        ariles(visitor, arilesGetParameters(visitor)); \
                    } \
                    \
                    virtual const typename t_Visitor::Parameters & arilesGetParameters(const t_Visitor &visitor) const \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        return(visitor.getDefaultParameters()); \
                    } \
            };


        ARILES_VISITOR_BASE(Base, ARILES_EMPTY_MACRO)
        ARILES_VISITOR_BASE(ConstBase, const)

        #undef ARILES_VISITOR_BASE
    }
}
