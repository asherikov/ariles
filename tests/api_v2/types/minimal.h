/**
    @file
    @author  Alexander Sherikov
    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableMinimal : public ariles::DefaultBase
    {
#define ARILES_ENTRIES(v) ARILES_TYPED_ENTRY(v, integer_member, int)
#include ARILES_INITIALIZE
    };
}  // namespace ariles_tests
