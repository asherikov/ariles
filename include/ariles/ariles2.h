/**
    @file
    @author  Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "internal/helpers.h"
#include "visitors/defaults.h"
#include "visitors/finalize.h"
#include "visitors/compare.h"
#include "visitors/count.h"
#include "visitors/read.h"
#include "visitors/write.h"


#define ARILES_API_VERSION 2

// These defines are always necessary
#define ARILES_TYPED_ENTRY_(entry, type) ARILES_TYPED_NAMED_ENTRY(type, entry##_, #entry)
#define ARILES_TYPED_ENTRY(entry, type)  ARILES_TYPED_NAMED_ENTRY(type, entry, #entry)


#ifndef ARILES_DISABLE
#   define ARILES_ENABLED


    #define ARILES_ENTRY_(entry)     ARILES_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_ENTRY(entry)      ARILES_NAMED_ENTRY(entry, #entry)



    #define ARILES_METHODS_WITH_ARG(Visitor, Parameters, Qualifier) \
        template<class t_Extra> \
            void ariles(Visitor &visitor, \
                        t_Extra & extra, \
                        Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            arilesVisit(visitor, extra, param); \
        } \
        template<class t_Extra> \
            void ariles(Visitor &visitor, \
                        t_Extra & extra) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            ariles(visitor, extra, visitor.default_parameters_); \
        }


    #define ARILES_METHODS(Visitor, Parameters, Qualifier) \
        virtual void ariles( \
                Visitor &visitor, \
                const std::string & name, \
                Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            visitor.startRoot(*this, param); \
            arilesEntryApply(visitor, *this, name, param); \
            visitor.finishRoot(*this, param); \
        } \
        virtual void arilesApply(   Visitor &visitor, \
                                    Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            this->arilesVisit(visitor, param); \
        }

    /**
     * Some visitors, e.g., entry counter, do not have virtual methods in the
     * base, so the following methods must be defined inside the derived
     * classes.
     */
    #define ARILES_NONVIRTUAL_METHODS(Visitor, Parameters, Qualifier) \
        void arilesApply(Visitor &visitor) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            arilesApply(visitor, visitor.default_parameters_); \
        } \
        void ariles(Visitor &visitor) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            ariles(visitor, visitor.default_parameters_); \
        } \
        void ariles(Visitor &visitor, Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            ariles(visitor, this->arilesDefaultID(), param); \
        }


    // ----------------------------



    namespace ariles
    {
        class ARILES_VISIBILITY_ATTRIBUTE Base
            :   public ariles::defaults::Base<ariles::Base>,
                public ariles::finalize::Base<ariles::Base>,
                public ariles::compare::Base<ariles::Base>,
                public ariles::count::Base<ariles::Base>,
                public ariles::read::Base<ariles::Base>,
                public ariles::write::Base<ariles::Base>
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~Base() {}
                Base() {}


            public:
                using ariles::defaults::Base<ariles::Base>::arilesApply;
                using ariles::finalize::Base<ariles::Base>::arilesApply;
                using ariles::read::Base<ariles::Base>::arilesApply;
                using ariles::write::Base<ariles::Base>::arilesApply;

                using ariles::defaults::Base<ariles::Base>::ariles;
                using ariles::finalize::Base<ariles::Base>::ariles;
                using ariles::read::Base<ariles::Base>::ariles;
                using ariles::write::Base<ariles::Base>::ariles;



                virtual const std::string & arilesDefaultID() const = 0;


                #define ARILES_BASE_METHODS(Qualifier) \
                    template <class t_Visitor> \
                        void arilesApply( \
                                const typename ariles::utils::DecayConst<t_Visitor>::Type::VisitorIndicatorType * = NULL) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor; \
                        arilesApply(visitor); \
                    }

                ARILES_BASE_METHODS(ARILES_EMPTY_MACRO);
                ARILES_BASE_METHODS(const);

                #undef ARILES_BASE_METHODS
        };
    }

// #   include "adapters/basic.h"
// #   include "types.h"

#else

#   define ARILES_DISABLED

    namespace ariles
    {
        // Some classes may inherit from this
        class ARILES_VISIBILITY_ATTRIBUTE Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of derived
                 * classes via base pointer.
                 */
                ~Base() {}
        };
    }

#endif
