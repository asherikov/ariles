/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"
#include "process.h"

namespace ariles
{
    namespace postprocess
    {
        class Parameters
        {
        };


        class ARILES_VISIBILITY_ATTRIBUTE Visitor
            : public ariles::process::Visitor<postprocess::Visitor, postprocess::Parameters>
        {
        };


        class ARILES_VISIBILITY_ATTRIBUTE Base
            : public visitor::Base<const postprocess::Visitor>
        {
            public:
        };


#ifndef ARILES_METHODS_postprocess
#   define ARILES_METHODS_postprocess ARILES_METHODS(postprocess, const ariles::postprocess::Visitor, ARILES_EMPTY_MACRO)
#endif
    }


    typedef postprocess::Visitor PostProcess;
}
