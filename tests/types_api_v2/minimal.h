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
    class ConfigurableMinimal : public ariles::Base
    {
        #define ARILES_DEFAULT_ID "ConfigurableEntryName"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY(integer_member, int)
        #include ARILES_INITIALIZE
    };
}
