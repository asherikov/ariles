/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "writer.h"

namespace ariles
{
    namespace bridge
    {
        namespace rapidjson
        {
            namespace jsonnet
            {
                // API symmetry is needed for some tests
                typedef rapidjson::Writer Writer;
            }
        }
    }
}
