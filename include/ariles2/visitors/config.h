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
#include "postread.h"
#include "prewrite.h"
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
            ariles2::preread::Visitor::Parameters preread_;
            typename t_Reader::Parameters read_;
            ariles2::postread::Visitor::Parameters postread_;


        public:
            Parameters(const bool override_parameters = true)
              : preread_(override_parameters), read_(override_parameters), postread_(override_parameters)
            {
            }

            Parameters(const typename t_Reader::Parameters &param, const bool override_parameters = true)
              : preread_(override_parameters), postread_(override_parameters)
            {
                this->read_ = param;
            }

            Parameters(
                    const ariles2::preread::Visitor::Parameters &preread,
                    const typename t_Reader::Parameters &reader,
                    const ariles2::postread::Visitor::Parameters &postread)
            {
                this->preread_ = preread;
                this->read_ = reader;
                this->postread_ = postread;
            }
        };


        template <class t_Reader>
        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public visitor::Base<visitor::GenericVisitor, Parameters<t_Reader> >
        {
        public:
            typedef cfgread::Parameters<t_Reader> Parameters;


        public:
            ariles2::preread::Visitor preread_;
            t_Reader read_;
            ariles2::postread::Visitor postread_;


        public:
            template <class t_Initializer>
            Visitor(t_Initializer &initializer) : read_(initializer)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer>
            Visitor(const t_Initializer &initializer) : read_(initializer)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 &initializer0, const t_Initializer1 &initializer1)
              : read_(initializer0, initializer1)
            {
                ARILES2_TRACE_FUNCTION;
            }


            using visitor::Base<visitor::GenericVisitor, Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters getParameters(const t_Ariles &ariles_class) const
            {
                // static variable is potentially unsafe
                return (Parameters(
                        ariles_class.arilesGetParameters(preread_),
                        ariles_class.arilesGetParameters(read_),
                        ariles_class.arilesGetParameters(postread_)));
            }


            template <class t_Entry>
            void visit(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);
                ARILES2_TRACE_TYPE(entry);
                ariles2::apply(preread_, entry, name, param.preread_);
                ariles2::apply(read_, entry, name, param.read_);
                ariles2::apply(postread_, entry, name, param.postread_);
            }


            const t_Reader &getReader() const
            {
                return (read_);
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
            ariles2::prewrite::Visitor::Parameters prewrite_;
            typename t_Writer::Parameters write_;


        public:
            Parameters(const bool override_parameters = true)
              : prewrite_(override_parameters), write_(override_parameters)
            {
            }

            Parameters(const typename t_Writer::Parameters &param, const bool override_parameters = true)
              : prewrite_(override_parameters)
            {
                this->write_ = param;
            }

            Parameters(
                    const ariles2::prewrite::Visitor::Parameters &prewrite,
                    const typename t_Writer::Parameters &writer)
            {
                this->prewrite_ = prewrite;
                this->write_ = writer;
            }
        };


        template <class t_Writer>
        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public visitor::Base<visitor::GenericVisitor, Parameters<t_Writer> >
        {
        public:
            typedef cfgwrite::Parameters<t_Writer> Parameters;


        public:
            ariles2::prewrite::Visitor prewrite_;
            t_Writer write_;


        public:
            template <class t_Initializer>
            Visitor(t_Initializer &initializer) : write_(initializer)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer>
            Visitor(const t_Initializer &initializer) : write_(initializer)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 *initializer0, const t_Initializer1 &initializer1)
              : write_(initializer0, initializer1)
            {
                ARILES2_TRACE_FUNCTION;
            }

            template <class t_Initializer0, class t_Initializer1>
            Visitor(t_Initializer0 &initializer0, const t_Initializer1 &initializer1)
              : write_(initializer0, initializer1)
            {
                ARILES2_TRACE_FUNCTION;
            }


            using visitor::Base<visitor::GenericVisitor, Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters getParameters(const t_Ariles &ariles_class) const
            {
                // static variable is potentially unsafe
                return (Parameters(
                        ariles_class.arilesGetParameters(prewrite_), ariles_class.arilesGetParameters(write_)));
            }


            template <class t_Entry>
            void visit(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);
                ARILES2_TRACE_TYPE(entry);
                ariles2::apply(prewrite_, entry, name, param.prewrite_);
                ariles2::apply(write_, entry, name, param.write_);
            }


            const t_Writer &getWriter() const
            {
                return (write_);
            }


            t_Writer &getWriter()
            {
                return (write_);
            }
        };
    }  // namespace cfgwrite
}  // namespace ariles2
