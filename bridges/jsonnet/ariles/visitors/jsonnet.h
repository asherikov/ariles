/**
    @file
    @author Alexander Sherikov
    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/

#pragma once

#define ARILES_VISITOR_INCLUDED_jsonnet


#include <ariles/internal/helpers.h>
#include <ariles/visitors/config.h>

#include "./jsonnet/reader.h"


namespace ariles
{
    template<class t_ParentVisitor>
        struct ARILES_VISIBILITY_ATTRIBUTE jsonnet
    {
        typedef ariles::cfgread::Visitor<bridge::jsonnet::Reader<typename t_ParentVisitor::ReaderBase> > Reader;
        typedef ariles::cfgwrite::Visitor<typename t_ParentVisitor::WriterBase> Writer;
    };
}
