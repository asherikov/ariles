/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Inclusion of this file results in generation of functions which
    read and write entries 'ARILES_ENTRIES' defined in the including
    header from / to a configuration file.
*/


#ifdef ARILES_ENABLED

    #ifndef ARILES_DOXYGEN_PROCESSING

    public:
        using ariles::Base::arilesGetParameters;
        using ariles::Base::arilesVirtualVisit;
        using ariles::Base::ariles;


        #ifdef ARILES_ENTRIES

            #define ARILES_NAMED_ENTRY(entry, name)     arilesEntryApply(visitor, entry, name, parameters);
            #define ARILES_PARENT(entry)                entry::arilesVisit(visitor, parameters);

            template<class t_Visitor, class t_Parameters>
            void arilesVisit(t_Visitor &visitor, const t_Parameters &parameters)
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
            }


            template<class t_Visitor, class t_Parameters>
            void arilesVisit(t_Visitor &visitor, const t_Parameters &parameters) const
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
            }

            #undef ARILES_PARENT
            #undef ARILES_NAMED_ENTRY



            #define ARILES_NAMED_ENTRY(entry, name)     arilesEntryApply(visitor, entry, other.entry, name, parameters);
            #define ARILES_PARENT(entry)                entry::arilesVisit(visitor, other, parameters);

            template<class t_Visitor, class t_Parameters, class t_Other>
                void arilesVisit(const t_Visitor &visitor, t_Other & other, const t_Parameters &parameters) const
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(other);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_MACRO_SUBSTITUTE(ARILES_ENTRIES)
            }

            #undef ARILES_PARENT
            #undef ARILES_NAMED_ENTRY


            #undef ARILES_TYPED_NAMED_ENTRY
        #endif
    #endif


    public:
        // Define node name
        #ifdef ARILES_DEFAULT_ID
            const std::string & arilesDefaultID() const
            {
                static const std::string name(ARILES_DEFAULT_ID);
                return (name);
            }
        #endif


        ARILES_METHODS(ariles::read::Visitor, ARILES_EMPTY_MACRO)

        ARILES_METHODS(ariles::write::Visitor, const)

        ARILES_METHODS(const ariles::defaults::Visitor, ARILES_EMPTY_MACRO)

        ARILES_METHODS(const ariles::finalize::Visitor, ARILES_EMPTY_MACRO)

        ARILES_METHODS(ariles::count::Visitor, const)

        ARILES_NONVIRTUAL_METHODS(ariles::count::Visitor, const)


        ARILES_METHODS_WITH_ARG(const ariles::compare::Visitor, const)


#endif //ARILES_ENABLED

#undef ARILES_DEFAULT_ID
