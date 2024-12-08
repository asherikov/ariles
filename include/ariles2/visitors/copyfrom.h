/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

/**
@defgroup copyfrom CopyFrom

@brief Copy data from non-ariles classes.
*/

namespace ariles2
{
    /// @ingroup copyfrom
    namespace copyfrom
    {
        class ARILES2_VISIBILITY_PUBLIC Parameters : public visitor::Parameters
        {
        public:
            bool deep_copy_;

        public:
            explicit Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
                deep_copy_ = true;
            }
        };


        class ARILES2_VISIBILITY_PUBLIC Visitor : public visitor::Base<Visitor, copyfrom::Parameters>
        {
        public:
            using Parameters = copyfrom::Parameters;


        public:
            template <class t_Left, class t_Right>
            void visit(t_Left &left, const t_Right &right, const std::string &name, const Parameters &param)
            {
                CPPUT_TRACE_FUNCTION;
                try
                {
                    this->visitMapEntry(left, right, name, param);
                }
                catch (std::exception &e)
                {
                    CPPUT_THROW("Copying failed: ", e.what());
                }
            }


            template <class t_Left, class t_Right>
            void visitMapEntry(t_Left &left, const t_Right &right, const std::string &name, const Parameters &param)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(name);
                CPPUT_TRACE_TYPE(left);
                CPPUT_TRACE_TYPE(right);

                try
                {
                    apply_copyfrom(*this, left, right, param);
                }
                catch (const std::exception &e)
                {
                    CPPUT_THROW("entry: ", name, " // ", e.what());
                }
            }
        };


        class ARILES2_VISIBILITY_PUBLIC Base
        {
        };


#define ARILES2_NAMED_ENTRY_copyfrom(v, entry, name) visitor.visitMapEntry(entry, other.name, #name, parameters);
#define ARILES2_PARENT_copyfrom(v, entry) entry::arilesVisit(visitor, other, parameters);

#define ARILES2_VISIT_copyfrom                                                                                         \
    template <class t_Other>                                                                                           \
    void arilesVisit(                                                                                                  \
            ariles2::copyfrom::Visitor &visitor,                                                                       \
            const t_Other &other,                                                                                      \
            const typename ariles2::copyfrom::Visitor::Parameters &parameters)                                         \
    {                                                                                                                  \
        CPPUT_UNUSED_ARG(visitor);                                                                                     \
        CPPUT_UNUSED_ARG(other);                                                                                       \
        CPPUT_UNUSED_ARG(parameters);                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        ARILES2_ENTRIES(copyfrom)                                                                                      \
    }

#define ARILES2_METHODS_copyfrom                                                                                       \
    const ariles2::copyfrom::Visitor::Parameters &arilesGetParameters(const ariles2::copyfrom::Visitor &visitor) const \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        return (visitor.getDefaultParameters());                                                                       \
    }
#define ARILES2_BASE_METHODS_copyfrom
    }  // namespace copyfrom


    /// @ingroup copyfrom
    using CopyFrom = copyfrom::Visitor;
}  // namespace ariles2
