/**
    @file
    @author  Alexander Sherikov
    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
*/


#define ARILES_NAMED_ENTRY_generic_parent(v, entry, name)
#define ARILES_PARENT_generic_parent(v, entry) entry::arilesVisit(visitor, parameters);

#define ARILES_VISIT_generic_parent                                                                                    \
    template <class t_Visitor>                                                                                         \
    void arilesVisitParents(                                                                                           \
            t_Visitor &visitor,                                                                                        \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES_IS_BASE_ENABLER(ariles::visitor::GenericVisitor, t_Visitor))                                        \
    {                                                                                                                  \
        ARILES_UNUSED_ARG(visitor);                                                                                    \
        ARILES_UNUSED_ARG(parameters);                                                                                 \
        ARILES_TRACE_FUNCTION;                                                                                         \
        ARILES_ENTRIES(generic_parent)                                                                                 \
    }                                                                                                                  \
    template <class t_Visitor>                                                                                         \
    void arilesVisitParents(                                                                                           \
            t_Visitor &visitor,                                                                                        \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES_IS_BASE_ENABLER(ariles::visitor::GenericVisitor, t_Visitor)) const                                  \
    {                                                                                                                  \
        ARILES_UNUSED_ARG(visitor);                                                                                    \
        ARILES_UNUSED_ARG(parameters);                                                                                 \
        ARILES_TRACE_FUNCTION;                                                                                         \
        ARILES_ENTRIES(generic_parent)                                                                                 \
    }
