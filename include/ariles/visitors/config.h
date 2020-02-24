/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
    @todo compound visitor
*/

#pragma once

#include "defaults.h"
#include "finalize.h"
#include "read.h"
#include "write.h"

#ifndef ARILES_API_VERSION
#   define ARILES_API_VERSION 1
#endif

namespace ariles
{
    namespace cfgread
    {
#if 1 == ARILES_API_VERSION
        template<class t_Reader>
            class Visitor : public t_Reader
        {
            public:
                template<class t_Initializer>
                    Visitor(t_Initializer &initializer) : t_Reader(initializer)
                {
                    ARILES_TRACE_FUNCTION;
                }

                template<class t_Initializer>
                    Visitor(const t_Initializer &initializer) : t_Reader(initializer)
                {
                    ARILES_TRACE_FUNCTION;
                }
        };
#endif

#if 2 == ARILES_API_VERSION
        template<class t_Reader>
            class Parameters
        {
            public:
                ariles::defaults::Visitor::Parameters parameters0_;
                typename t_Reader::Parameters parameters1_;
                ariles::finalize::Visitor::Parameters parameters2_;


            public:
                Parameters()
                {
                }

                Parameters(const int &param)
                {
                    this->parameters1_ = param;
                }

                Parameters(const typename t_Reader::Parameters &param)
                {
                    this->parameters1_ = param;
                }

                Parameters(const ariles::defaults::Visitor::Parameters &parameters0,
                           const typename t_Reader::Parameters &parameters1,
                           const ariles::finalize::Visitor::Parameters &parameters2)
                {
                    this->parameters0_ = parameters0;
                    this->parameters1_ = parameters1;
                    this->parameters2_ = parameters2;
                }
        };


        template<class t_Reader>
            class Visitor : public visitor::VisitorBase< Parameters<t_Reader> >
        {
            public:
                typedef cfgread::Parameters<t_Reader> Parameters;


            public:
                ariles::defaults::Visitor visitors0_;
                t_Reader visitors1_;
                ariles::finalize::Visitor visitors2_;


            public:
                template<class t_Initializer>
                    Visitor(t_Initializer &initializer) : visitors1_(initializer)
                {
                    ARILES_TRACE_FUNCTION;
                }

                template<class t_Initializer>
                    Visitor(const t_Initializer &initializer) : visitors1_(initializer)
                {
                    ARILES_TRACE_FUNCTION;
                }


                using visitor::VisitorBase<Parameters>::getDefaultParameters;

                template<class t_Ariles>
                    const Parameters getParameters(const t_Ariles & ariles_class) const
                {
                    // static variable is potentially unsafe
                    return (Parameters(
                                ariles_class.arilesGetParameters(visitors0_),
                                ariles_class.arilesGetParameters(visitors1_),
                                ariles_class.arilesGetParameters(visitors2_)));
                }


                using visitor::VisitorBase<Parameters>::startRoot;
                using visitor::VisitorBase<Parameters>::finishRoot;


                template<class t_Entry>
                    void operator()(
                            t_Entry & entry,
                            const std::string & name,
                            const Parameters & param)
                {
                    ARILES_TRACE_FUNCTION;
                    ARILES_TRACE_ENTRY(name);
                    ARILES_TRACE_TYPE(entry);
                    ariles::apply(visitors0_, entry, name, param.parameters0_);
                    ariles::apply(visitors1_, entry, name, param.parameters1_);
                    ariles::apply(visitors2_, entry, name, param.parameters2_);
                }


                const t_Reader & getReader() const
                {
                    return (visitors1_);
                }
        };
#endif
    }
}


namespace ariles
{
    namespace cfgwrite
    {
        template<class t_Writer>
            class Visitor : public t_Writer
        {
            public:
                template<class t_Initializer>
                    Visitor(t_Initializer &initializer) : t_Writer(initializer)
                {
                    ARILES_TRACE_FUNCTION;
                }

                template<class t_Initializer>
                    Visitor(const t_Initializer &initializer) : t_Writer(initializer)
                {
                    ARILES_TRACE_FUNCTION;
                }

                template<class t_Initializer0, class t_Initializer1>
                    Visitor(t_Initializer0 * initializer0, const t_Initializer1 & initializer1) : t_Writer(initializer0, initializer1)
                {
                    ARILES_TRACE_FUNCTION;
                }

                const t_Writer & getWriter() const
                {
                    return (*this);
                }
        };
    }
}
