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


#define ARILES_VISIT_postprocess
#define ARILES_METHODS_postprocess ARILES_METHODS(postprocess, const, ARILES_EMPTY_MACRO)
    }  // namespace postprocess


    typedef postprocess::Visitor PostProcess;
}  // namespace ariles2
