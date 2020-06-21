/**
    @file
    @author Alexander Sherikov
    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/

/**
@defgroup jsonnet Jsonnet

@brief Preprocessing wrapper for json visitors, see https://jsonnet.org/.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_jsonnet


#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>

#include "./jsonnet/reader.h"


namespace ariles2
{
    /**
     * @brief Jsonnet visitor wrapper.
     * @ingroup jsonnet
     */
    template <class t_ParentVisitor>
    struct ARILES2_VISIBILITY_ATTRIBUTE jsonnet
    {
        typedef ariles2::cfgread::Visitor<ns_jsonnet::Reader<typename t_ParentVisitor::ReaderBase> > Reader;
        typedef ariles2::cfgwrite::Visitor<typename t_ParentVisitor::WriterBase> Writer;
    };
}  // namespace ariles2
