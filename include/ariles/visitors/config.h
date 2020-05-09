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
#include "postprocess.h"
#include "preprocess.h"
#include "read.h"
#include "write.h"


namespace ariles
{
    namespace cfgread
    {
        template <class t_Reader>
        class ARILES_VISIBILITY_ATTRIBUTE Parameters
        {
        public:
            ariles::defaults::Visitor::Parameters defaults_parameters_;
            typename t_Reader::Parameters reader_parameters_;
            ariles::postprocess::Visitor::Parameters postprocess_parameters_;


        public:
            Parameters()
            {
            }

            Parameters(const int &param)
            {
                this->reader_parameters_ = param;
            }

            Parameters(const typename t_Reader::Parameters &param)
            {
                this->reader_parameters_ = param;
            }

            Parameters(
                    const ariles::defaults::Visitor::Parameters &defaults_parameters,
                    const typename t_Reader::Parameters &reader_parameters,
                    const ariles::postprocess::Visitor::Parameters &postprocess_parameters)
            {
                this->defaults_parameters_ = defaults_parameters;
                this->reader_parameters_ = reader_parameters;
                this->postprocess_parameters_ = postprocess_parameters;
            }
        };


        template <class t_Reader>
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public visitor::Base<Parameters<t_Reader> >
        {
        public:
            typedef cfgread::Parameters<t_Reader> Parameters;


        public:
            ariles::defaults::Visitor defaults_visitor_;
            t_Reader reader_visitor_;
            ariles::postprocess::Visitor postprocess_visitor_;


        public:
            template <class t_Initializer>
            Visitor(t_Initializer &initializer) : reader_visitor_(initializer)
            {
                ARILES_TRACE_FUNCTION;
            }

            template <class t_Initializer>
            Visitor(const t_Initializer &initializer) : reader_visitor_(initializer)
            {
                ARILES_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 &initializer0, const t_Initializer1 &initializer1)
              : reader_visitor_(initializer0, initializer1)
            {
                ARILES_TRACE_FUNCTION;
            }


            using visitor::Base<Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters getParameters(const t_Ariles &ariles_class) const
            {
                // static variable is potentially unsafe
                return (Parameters(
                        ariles_class.arilesGetParameters(defaults_visitor_),
                        ariles_class.arilesGetParameters(reader_visitor_),
                        ariles_class.arilesGetParameters(postprocess_visitor_)));
            }


            template <class t_Entry>
            void start(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES_TRACE_FUNCTION;
                this->operator()(entry, name, param);
            }


            template <class t_Entry>
            void operator()(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_ENTRY(name);
                ARILES_TRACE_TYPE(entry);
                ariles::apply(defaults_visitor_, entry, name, param.defaults_parameters_);
                ariles::apply(reader_visitor_, entry, name, param.reader_parameters_);
                ariles::apply(postprocess_visitor_, entry, name, param.postprocess_parameters_);
            }


            const t_Reader &getReader() const
            {
                return (reader_visitor_);
            }
        };
    }  // namespace cfgread
}  // namespace ariles


namespace ariles
{
    namespace cfgwrite
    {
        template <class t_Writer>
        class ARILES_VISIBILITY_ATTRIBUTE Parameters
        {
        public:
            ariles::preprocess::Visitor::Parameters preprocess_parameters_;
            typename t_Writer::Parameters writer_parameters_;


        public:
            Parameters()
            {
            }

            Parameters(const int &param)
            {
                this->writer_parameters_ = param;
            }

            Parameters(const typename t_Writer::Parameters &param)
            {
                this->writer_parameters_ = param;
            }

            Parameters(
                    const ariles::preprocess::Visitor::Parameters &preprocess_parameters,
                    const typename t_Writer::Parameters &writer_parameters)
            {
                this->preprocess_parameters_ = preprocess_parameters;
                this->writer_parameters_ = writer_parameters;
            }
        };


        template <class t_Writer>
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public visitor::Base<Parameters<t_Writer> >
        {
        public:
            typedef cfgwrite::Parameters<t_Writer> Parameters;


        public:
            ariles::preprocess::Visitor preprocess_visitor_;
            t_Writer writer_visitor_;


        public:
            template <class t_Initializer>
            Visitor(t_Initializer &initializer) : writer_visitor_(initializer)
            {
                ARILES_TRACE_FUNCTION;
            }

            template <class t_Initializer>
            Visitor(const t_Initializer &initializer) : writer_visitor_(initializer)
            {
                ARILES_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 *initializer0, const t_Initializer1 &initializer1)
              : writer_visitor_(initializer0, initializer1)
            {
                ARILES_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 &initializer0, const t_Initializer1 &initializer1)
              : writer_visitor_(initializer0, initializer1)
            {
                ARILES_TRACE_FUNCTION;
            }


            using visitor::Base<Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters getParameters(const t_Ariles &ariles_class) const
            {
                // static variable is potentially unsafe
                return (Parameters(
                        ariles_class.arilesGetParameters(preprocess_visitor_),
                        ariles_class.arilesGetParameters(writer_visitor_)));
            }


            template <class t_Entry>
            void start(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES_TRACE_FUNCTION;
                this->operator()(entry, name, param);
            }


            template <class t_Entry>
            void operator()(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES_TRACE_FUNCTION;
                ARILES_TRACE_ENTRY(name);
                ARILES_TRACE_TYPE(entry);
                ariles::apply(preprocess_visitor_, entry, name, param.preprocess_parameters_);
                ariles::apply(writer_visitor_, entry, name, param.writer_parameters_);
            }


            const t_Writer &getWriter() const
            {
                return (writer_visitor_);
            }


            t_Writer &getWriter()
            {
                return (writer_visitor_);
            }
        };
    }  // namespace cfgwrite
}  // namespace ariles
