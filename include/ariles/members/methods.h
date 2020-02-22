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


        #ifdef ARILES_ENTRIES

            #define ARILES_NAMED_ENTRY(entry, name)     visitor(entry, name, parameters);
            #define ARILES_PARENT(entry)                entry::arilesVisit(visitor, parameters);

            template<class t_Visitor>
                void arilesVisit(t_Visitor &visitor, const typename t_Visitor::Parameters &parameters)
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_ENTRIES
            }


            template<class t_Visitor>
                void arilesVisit(t_Visitor &visitor, const typename t_Visitor::Parameters &parameters) const
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_ENTRIES
            }

            #undef ARILES_PARENT
            #undef ARILES_NAMED_ENTRY



            #define ARILES_NAMED_ENTRY(entry, name)     visitor(entry, other.entry, name, parameters);
            #define ARILES_PARENT(entry)                entry::arilesVisit(visitor, other, parameters);

            template<class t_Visitor, class t_Other>
                void arilesVisit(
                        t_Visitor &visitor,
                        const t_Other & other,
                        const typename t_Visitor::Parameters &parameters) const
            {
                ARILES_UNUSED_ARG(visitor);
                ARILES_UNUSED_ARG(other);
                ARILES_UNUSED_ARG(parameters);
                ARILES_TRACE_FUNCTION;
                ARILES_ENTRIES
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


        #ifndef ARILES_VISITORS
            #define ARILES_VISITORS ARILES_DEFAULT_VISITORS
        #endif

        #define ARILES_VISITOR(visitor) ARILES_METHODS_##visitor

        ARILES_VISITORS

        #undef ARILES_VISITOR
        #undef ARILES_VISITORS


#endif //ARILES_ENABLED

#undef ARILES_DEFAULT_ID
