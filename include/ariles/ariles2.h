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

// must be first
#ifndef ARILES_API_VERSION
#   define ARILES_API_VERSION 2
#endif


#include "internal/helpers.h"
#include "visitors/defaults.h"
#include "visitors/finalize.h"
#include "visitors/compare.h"
#include "visitors/count.h"
#include "visitors/read.h"
#include "visitors/write.h"


// These defines are always necessary
#define ARILES_TYPED_ENTRY_(entry, type) ARILES_TYPED_NAMED_ENTRY(type, entry##_, #entry)
#define ARILES_TYPED_ENTRY(entry, type)  ARILES_TYPED_NAMED_ENTRY(type, entry, #entry)


#ifndef ARILES_DISABLE
#   define ARILES_ENABLED


    #define ARILES_INITIALIZE  "ariles/members/all.h"

    #define ARILES_ENTRY_(entry)     ARILES_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_ENTRY(entry)      ARILES_NAMED_ENTRY(entry, #entry)



    #define ARILES_METHODS_WITH_ARG(Visitor, Qualifier) \
        template<class t_Extra> \
            void ariles(Visitor &visitor, \
                        t_Extra & extra, \
                        const ariles::utils::DecayConst<Visitor>::Type::Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            try \
            { \
                visitor.startRoot(*this, param); \
                arilesEntryApply(visitor, *this, extra, arilesDefaultID(), param); \
                visitor.finishRoot(*this, param); \
            } \
            catch (std::exception &e) \
            { \
                if (true == param.throw_on_error_) \
                { \
                    ARILES_THROW(std::string("Comparison failed: ") + e.what()); \
                } \
                visitor.equal_ = false; \
            } \
        } \
        template<class t_Extra> \
            void ariles(Visitor &visitor, \
                        t_Extra & extra) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            ariles(visitor, extra, arilesGetParameters(visitor)); \
        } \
        const ariles::utils::DecayConst<Visitor>::Type::Parameters & \
            arilesGetParameters(const ariles::utils::DecayConst<Visitor>::Type &visitor) const \
        { \
            ARILES_TRACE_FUNCTION; \
            return(visitor.getDefaultParameters()); \
        }


    #define ARILES_METHODS(Visitor, Qualifier) \
        virtual void ariles( \
                Visitor &visitor, \
                const std::string & name, \
                const ariles::utils::DecayConst<Visitor>::Type::Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            visitor.startRoot(*this, param); \
            arilesEntryApply(visitor, *this, name, param); \
            visitor.finishRoot(*this, param); \
        } \
        virtual void arilesVirtualVisit(Visitor &visitor, \
                                        const ariles::utils::DecayConst<Visitor>::Type::Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            this->arilesVisit(visitor, param); \
        }

    /**
     * Some visitors, e.g., entry counter, do not have virtual methods in the
     * base, so the following methods must be defined inside the derived
     * classes.
     */
    #define ARILES_NONVIRTUAL_METHODS(Visitor, Qualifier) \
        void arilesVirtualVisit(Visitor &visitor) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            arilesVirtualVisit(visitor, arilesGetParameters(visitor)); \
        } \
        void ariles(Visitor &visitor) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            ariles(visitor, arilesGetParameters(visitor)); \
        } \
        void ariles(Visitor &visitor, const ariles::utils::DecayConst<Visitor>::Type::Parameters &param) Qualifier \
        { \
            ARILES_TRACE_FUNCTION; \
            ariles(visitor, this->arilesDefaultID(), param); \
        } \
        const ariles::utils::DecayConst<Visitor>::Type::Parameters & \
            arilesGetParameters(const ariles::utils::DecayConst<Visitor>::Type &visitor) const \
        { \
            ARILES_TRACE_FUNCTION; \
            return(visitor.getDefaultParameters()); \
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
                using ariles::defaults::Base<ariles::Base>::arilesVirtualVisit;
                using ariles::finalize::Base<ariles::Base>::arilesVirtualVisit;
                using ariles::read::Base<ariles::Base>::arilesVirtualVisit;
                using ariles::write::Base<ariles::Base>::arilesVirtualVisit;

                using ariles::defaults::Base<ariles::Base>::ariles;
                using ariles::finalize::Base<ariles::Base>::ariles;
                using ariles::read::Base<ariles::Base>::ariles;
                using ariles::write::Base<ariles::Base>::ariles;

                using ariles::defaults::Base<ariles::Base>::arilesGetParameters;
                using ariles::finalize::Base<ariles::Base>::arilesGetParameters;
                using ariles::read::Base<ariles::Base>::arilesGetParameters;
                using ariles::write::Base<ariles::Base>::arilesGetParameters;


                virtual const std::string & arilesDefaultID() const = 0;


                #define ARILES_BASE_METHODS(Qualifier) \
                    template <class t_Visitor> \
                        void arilesVirtualVisit( \
                                ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor; \
                        arilesVirtualVisit(visitor, arilesGetParameters(visitor)); \
                    } \
                    template <class t_Visitor> \
                        void ariles(ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor; \
                        ariles(visitor, arilesGetParameters(visitor)); \
                    } \
                    template <class t_Visitor, class t_Arg> \
                        void ariles( \
                                t_Arg & arg, \
                                ARILES_IS_BASE_DISABLER(ariles::visitor::Visitor, t_Arg), \
                                ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor(arg); \
                        ariles(visitor, arilesGetParameters(visitor)); \
                    } \
                    template <class t_Visitor, class t_Arg> \
                        void ariles( \
                                t_Arg & arg, \
                                const char *name, \
                                ARILES_IS_BASE_DISABLER(ariles::visitor::Visitor, t_Arg), \
                                ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor(arg); \
                        ariles(visitor, name, arilesGetParameters(visitor)); \
                    } \
                    template <class t_Visitor, class t_Arg> \
                        void ariles( \
                                const t_Arg & arg, \
                                ARILES_IS_BASE_DISABLER(ariles::visitor::Visitor, t_Arg), \
                                ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor(arg); \
                        ariles(visitor, arilesGetParameters(visitor)); \
                    } \
                    template <class t_Visitor, class t_Arg> \
                        void ariles( \
                                const t_Arg & arg, \
                                const char *name, \
                                ARILES_IS_BASE_DISABLER(ariles::visitor::Visitor, t_Arg), \
                                ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor(arg); \
                        ariles(visitor, name, arilesGetParameters(visitor)); \
                    } \
                    template <class t_Visitor, class t_Arg> \
                        void ariles( \
                                t_Arg &arg, \
                                const typename t_Visitor::Parameters &param, \
                                ARILES_IS_BASE_DISABLER(ariles::visitor::Visitor, t_Arg), \
                                ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor(arg); \
                        ariles(visitor, param); \
                    } \
                    template <class t_Visitor> \
                        void ariles( \
                                const std::string &arg, \
                                const int flags, \
                                ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        t_Visitor visitor(arg); \
                        typename t_Visitor::Parameters param(flags); \
                        ariles(visitor, param); \
                    } \
                    template <class t_Visitor> \
                        void ariles( \
                                t_Visitor &visitor, \
                                const std::string &name, \
                                ARILES_IS_BASE_ENABLER(ariles::visitor::Visitor, t_Visitor)) Qualifier \
                    { \
                        ARILES_TRACE_FUNCTION; \
                        ariles(visitor, name, arilesGetParameters(visitor)); \
                    }


                ARILES_BASE_METHODS(ARILES_EMPTY_MACRO)
                ARILES_BASE_METHODS(const)

                #undef ARILES_BASE_METHODS
        };
    }

#   if 2 == ARILES_API_VERSION
#       include "adapters/basic.h"
#   endif

#else

#   define ARILES_DISABLED
#   define ARILES_INITIALIZE  "ariles/members/variables.h"

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
                Base() {}

            public:
                template<class t_First>
                    void ariles() const
                {
                }

                template<class t_First, class t_Second>
                    void ariles(const t_Second) const
                {
                }

                template<class t_First, class t_Second, class t_Third>
                    void ariles(const t_Second, const t_Third) const
                {
                }
        };
    }

#endif
