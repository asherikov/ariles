/**
    @file
    @author Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup protobuf3 Protocol Buffers [incomplete]

@brief Exchange data with protobuf classes.

Only trivial protobuf messages are supported: no repeated fields, no nested
messages, etc. The main limitation is that protobuf C++ API depends on both the
name and the type of message fields, which makes it impossible to handle either
via templates or preprocessor macro.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_protobuf3

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>
#include <ariles2/adapters/basic.h>


namespace ariles2
{
    namespace ns_protobuf3
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Base
        {
        };


#define ARILES2_NAMED_ENTRY_protobuf3_write(v, entry, name) other.set_##name(entry);
#define ARILES2_NAMED_ENTRY_protobuf3_read(v, entry, name)                                                             \
    visitor.visitMapEntry(entry, other.name(), #name, parameters);
#define ARILES2_PARENT_protobuf3_write(v, entry) entry::arilesVisit(visitor, other, parameters);
#define ARILES2_PARENT_protobuf3_read(v, entry) entry::arilesVisit(visitor, other, parameters);


#define ARILES2_VISIT_protobuf3                                                                                        \
    template <class t_Other>                                                                                           \
    void arilesVisit(                                                                                                  \
            ariles2::protobuf3::Writer &visitor,                                                                       \
            t_Other &other,                                                                                            \
            const typename ariles2::protobuf3::Writer::Parameters &parameters) const                                   \
    {                                                                                                                  \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(other);                                                                                     \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_ENTRIES(protobuf3_write)                                                                               \
    }                                                                                                                  \
    template <class t_Other>                                                                                           \
    void arilesVisit(                                                                                                  \
            ariles2::protobuf3::Reader &visitor,                                                                       \
            const t_Other &other,                                                                                      \
            const typename ariles2::protobuf3::Reader::Parameters &parameters)                                         \
    {                                                                                                                  \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(other);                                                                                     \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_ENTRIES(protobuf3_read)                                                                                \
    }


#define ARILES2_METHODS_protobuf3                                                                                      \
    const ariles2::protobuf3::Writer::Parameters &arilesGetParameters(const ariles2::protobuf3::Writer &visitor) const \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        return (visitor.getDefaultParameters());                                                                       \
    }                                                                                                                  \
    const ariles2::protobuf3::Reader::Parameters &arilesGetParameters(const ariles2::protobuf3::Reader &visitor) const \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        return (visitor.getDefaultParameters());                                                                       \
    }

#define ARILES2_BASE_METHODS_protobuf3
    }  // namespace ns_protobuf3
}  // namespace ariles2


#include "./protobuf3/reader.h"
#include "./protobuf3/writer.h"


namespace ariles2
{
    /**
     * @brief protobuf3 visitor.
     * @ingroup protobuf3
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE protobuf3
    {
        using Base = ns_protobuf3::Base;
        using Reader = ns_protobuf3::Reader;
        using Writer = ns_protobuf3::Writer;
    };
}  // namespace ariles2
