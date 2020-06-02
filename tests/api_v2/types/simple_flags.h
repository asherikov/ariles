/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <ariles2/extra.h>

namespace ariles_tests
{
    class ConfigurableFlags1 : public ariles2::SloppyBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE


    public:
        ariles2::read::Parameters getExpectedReadParameters() const
        {
            ariles2::read::Parameters parameters;

            parameters.sloppy_maps_ = true;
            parameters.sloppy_pairs_ = true;

            return (parameters);
        }

        ariles2::write::Parameters getExpectedWriteParameters() const
        {
            ariles2::write::Parameters parameters;

            parameters.sloppy_maps_ = true;
            parameters.sloppy_pairs_ = true;

            return (parameters);
        }
    };
}  // namespace ariles_tests
