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

namespace ariles2
{
    namespace postprocess
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters
        {
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public ariles2::process::Visitor<const postprocess::Visitor, postprocess::Parameters>
        {
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::Base<const postprocess::Visitor>
        {
        };


#define ARILES2_VISIT_postprocess
#define ARILES2_METHODS_postprocess ARILES2_METHODS(postprocess, const, ARILES2_EMPTY_MACRO)
#define ARILES2_BASE_METHODS_postprocess ARILES2_BASE_METHODS(postprocess)
    }  // namespace postprocess


    typedef postprocess::Visitor PostProcess;
}  // namespace ariles2
