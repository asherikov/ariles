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
    namespace preprocess
    {
        class Parameters
        {
        };


        class ARILES_VISIBILITY_ATTRIBUTE Visitor
          : public ariles::process::Visitor<const preprocess::Visitor, preprocess::Parameters>
        {
        };


        class ARILES_VISIBILITY_ATTRIBUTE Base : public entry::Base<const preprocess::Visitor>
        {
        };


#ifndef ARILES_METHODS_preprocess
#    define ARILES_METHODS_preprocess ARILES_METHODS(preprocess, const, ARILES_EMPTY_MACRO)
#endif
    }  // namespace preprocess


    typedef preprocess::Visitor PreProcess;
}  // namespace ariles
