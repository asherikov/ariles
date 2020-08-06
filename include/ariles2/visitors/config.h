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

/**
@defgroup config Configuration
@ingroup serialization

@brief Configuration visitors (perform pre- and post- processing during (de)serialization)
*/

namespace ariles2
{
    /// @ingroup config
    namespace cfgread
    {
        template <class t_Reader>
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters
        {
        public:
            ariles2::defaults::Visitor::Parameters defaults_parameters_;
            typename t_Reader::Parameters reader_parameters_;
            ariles2::postprocess::Visitor::Parameters postprocess_parameters_;


        public:
            Parameters(const bool override_parameters = true)
              : defaults_parameters_(override_parameters)
              , reader_parameters_(override_parameters)
              , postprocess_parameters_(override_parameters)
            {
            }

            Parameters(const typename t_Reader::Parameters &param, const bool override_parameters = true)
              : defaults_parameters_(override_parameters), postprocess_parameters_(override_parameters)
            {
                this->reader_parameters_ = param;
            }

            Parameters(
                    const ariles2::defaults::Visitor::Parameters &defaults_parameters,
                    const typename t_Reader::Parameters &reader_parameters,
                    const ariles2::postprocess::Visitor::Parameters &postprocess_parameters)
            {
                this->defaults_parameters_ = defaults_parameters;
                this->reader_parameters_ = reader_parameters;
                this->postprocess_parameters_ = postprocess_parameters;
            }
        };


        template <class t_Reader>
        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public visitor::Base<visitor::GenericVisitor, Parameters<t_Reader> >
        {
        public:
            typedef cfgread::Parameters<t_Reader> Parameters;


        public:
            ariles2::defaults::Visitor defaults_visitor_;
            t_Reader reader_visitor_;
            ariles2::postprocess::Visitor postprocess_visitor_;


        public:
            template <class t_Initializer>
            Visitor(t_Initializer &initializer) : reader_visitor_(initializer)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer>
            Visitor(const t_Initializer &initializer) : reader_visitor_(initializer)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 &initializer0, const t_Initializer1 &initializer1)
              : reader_visitor_(initializer0, initializer1)
            {
                ARILES2_TRACE_FUNCTION;
            }


            using visitor::Base<visitor::GenericVisitor, Parameters>::getDefaultParameters;

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
            void visit(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);
                ARILES2_TRACE_TYPE(entry);
                ariles2::apply(defaults_visitor_, entry, name, param.defaults_parameters_);
                ariles2::apply(reader_visitor_, entry, name, param.reader_parameters_);
                ariles2::apply(postprocess_visitor_, entry, name, param.postprocess_parameters_);
            }


            const t_Reader &getReader() const
            {
                return (reader_visitor_);
            }
        };
    }  // namespace cfgread
}  // namespace ariles2


namespace ariles2
{
    /// @ingroup config
    namespace cfgwrite
    {
        template <class t_Writer>
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters
        {
        public:
            ariles2::preprocess::Visitor::Parameters preprocess_parameters_;
            typename t_Writer::Parameters writer_parameters_;


        public:
            Parameters()
            {
            }

            Parameters(const typename t_Writer::Parameters &param)
            {
                this->writer_parameters_ = param;
            }

            Parameters(
                    const ariles2::preprocess::Visitor::Parameters &preprocess_parameters,
                    const typename t_Writer::Parameters &writer_parameters)
            {
                this->preprocess_parameters_ = preprocess_parameters;
                this->writer_parameters_ = writer_parameters;
            }
        };


        template <class t_Writer>
        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public visitor::Base<visitor::GenericVisitor, Parameters<t_Writer> >
        {
        public:
            typedef cfgwrite::Parameters<t_Writer> Parameters;


        public:
            ariles2::preprocess::Visitor preprocess_visitor_;
            t_Writer writer_visitor_;


        public:
            template <class t_Initializer>
            Visitor(t_Initializer &initializer) : writer_visitor_(initializer)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer>
            Visitor(const t_Initializer &initializer) : writer_visitor_(initializer)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 *initializer0, const t_Initializer1 &initializer1)
              : writer_visitor_(initializer0, initializer1)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 &initializer0, const t_Initializer1 &initializer1)
              : writer_visitor_(initializer0, initializer1)
            {
                ARILES2_TRACE_FUNCTION;
            }


            using visitor::Base<visitor::GenericVisitor, Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters getParameters(const t_Ariles &ariles_class) const
            {
                // static variable is potentially unsafe
                return (Parameters(
                        ariles_class.arilesGetParameters(preprocess_visitor_),
                        ariles_class.arilesGetParameters(writer_visitor_)));
            }


            template <class t_Entry>
            void visit(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);
                ARILES2_TRACE_TYPE(entry);
                ariles2::apply(preprocess_visitor_, entry, name, param.preprocess_parameters_);
                ariles2::apply(writer_visitor_, entry, name, param.writer_parameters_);
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
}  // namespace ariles2
