/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

enum SomeEnum
{
    UNDEFINED = 0,
    SOME_VALUE = 1,
    ANOTHER_VALUE = 2
};


#ifdef ARILES_ADAPTER_BETTER_ENUMS

#define BETTER_ENUMS_DEFAULT_CONSTRUCTOR(Enum)  \
    public:                                     \
        Enum() : _value(0) { }

#include "better_enum.h"

BETTER_ENUM(BetterEnum, int, UNDEFINED = 0, DEFINED_1, DEFINED_2)

#endif
