/**
    @file
    @author  Alexander Sherikov
    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
*/


#define ARILES2_NAMED_ENTRY_generic_parent(v, entry, name)
#define ARILES2_PARENT_generic_parent(v, entry) entry::arilesVisit(visitor, parameters);

#define ARILES2_VISIT_generic_parent                                                                                   \
    template <class t_Visitor, typename = ariles2::traits::is_visitor_t<t_Visitor>>                                    \
    void arilesVisitParents(t_Visitor &visitor, const typename t_Visitor::Parameters &parameters)                      \
    {                                                                                                                  \
        static_assert(                                                                                                 \
                std::is_base_of_v<ariles2::Ariles, typename std::decay<decltype(*this)>::type>,                        \
                "Class where ARILES2_INITIALIZE is included must inherit from an ariles class.");                      \
        CPPUT_UNUSED_ARG(visitor);                                                                                     \
        CPPUT_UNUSED_ARG(parameters);                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        ARILES2_ENTRIES(generic_parent)                                                                                \
    }                                                                                                                  \
    template <class t_Visitor, typename = ariles2::traits::is_visitor_t<t_Visitor>>                                    \
    void arilesVisitParents(t_Visitor &visitor, const typename t_Visitor::Parameters &parameters) const                \
    {                                                                                                                  \
        CPPUT_UNUSED_ARG(visitor);                                                                                     \
        CPPUT_UNUSED_ARG(parameters);                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        ARILES2_ENTRIES(generic_parent)                                                                                \
    }
