/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Extra ariles types, not required.
*/

#pragma once

#include "ariles.h"


#ifdef ARILES2_ENABLED

namespace ariles2
{
    namespace read
    {
        class SloppyParameters : public Parameters
        {
        public:
            SloppyParameters()
            {
                sloppy_maps_ = true;
                sloppy_pairs_ = true;
            }
        };

        class RelaxedSloppyParameters : public SloppyParameters
        {
        public:
            RelaxedSloppyParameters()
            {
                missing_entries_ = Parameters::MISSING_ENTRIES_ENABLE;
            }
        };
    }  // namespace read

    namespace write
    {
        class SloppyParameters : public Parameters
        {
        public:
            SloppyParameters()
            {
                sloppy_maps_ = true;
                sloppy_pairs_ = true;
            }
        };
    }  // namespace write


    class SloppyBase : public DefaultBase
    {
    public:
        using DefaultBase::arilesGetParameters;

        virtual const read::Visitor::Parameters &arilesGetParameters(const read::Visitor &) const
        {
            ARILES2_TRACE_FUNCTION;
            const static read::SloppyParameters parameters;
            return (parameters);
        }

        virtual const write::Visitor::Parameters &arilesGetParameters(const write::Visitor &) const
        {
            ARILES2_TRACE_FUNCTION;
            const static write::SloppyParameters parameters;
            return (parameters);
        }
    };


    class RelaxedSloppyBase : public SloppyBase
    {
    public:
        using DefaultBase::arilesGetParameters;

        virtual const read::Visitor::Parameters &arilesGetParameters(const read::Visitor &) const
        {
            ARILES2_TRACE_FUNCTION;
            const static read::RelaxedSloppyParameters parameters;
            return (parameters);
        }
    };
}  // namespace ariles2

#else

namespace ariles2
{
    // Some classes may inherit from this
    typedef SloppyBase DefaultBase;
    typedef RelaxedSloppyBase DefaultBase;
}  // namespace ariles2

#endif
