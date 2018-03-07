/**
    @file
    @author Alexander Sherikov
    @author Jan Michalczyk

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#if defined(ARILES_ENABLED) || defined(ARILES_DISABLED)
    #error "This header must be included before config.h"
#endif


#include "../internal/helpers.h"
#include "../internal/reader_base.h"

#include ARILES_BRIDGE_yaml_cpp03_INCLUDE_HEADER

#include "./yaml_cpp03/reader.h"
#include "./yaml_cpp03/writer.h"


#define ARILES_YAML_NAMESPACE yaml_cpp03


// If something is stupid but it works, it is not stupid (c)
#ifndef ARILES_NAMESPACE_0
#   define ARILES_NAMESPACE_0 ARILES_YAML_NAMESPACE
#else
#   ifndef ARILES_NAMESPACE_1
#       define ARILES_NAMESPACE_1 ARILES_YAML_NAMESPACE
#   else
#       ifndef ARILES_NAMESPACE_2
#           define ARILES_NAMESPACE_2 ARILES_YAML_NAMESPACE
#       else
#           ifndef ARILES_NAMESPACE_3
#               define ARILES_NAMESPACE_3 ARILES_YAML_NAMESPACE
#           else
#               ifndef ARILES_NAMESPACE_4
#                   define ARILES_NAMESPACE_4 ARILES_YAML_NAMESPACE
#               else
#                   ifndef ARILES_NAMESPACE_5
#                       define ARILES_NAMESPACE_5 ARILES_YAML_NAMESPACE
#                   else
#                       ifndef ARILES_NAMESPACE_6
#                           define ARILES_NAMESPACE_6 ARILES_YAML_NAMESPACE
#                       else
#                           ifndef ARILES_NAMESPACE_7
#                               define ARILES_NAMESPACE_7 ARILES_YAML_NAMESPACE
#                           else
#                               error "Too many config namespaces."
#                           endif
#                       endif
#                   endif
#               endif
#           endif
#       endif
#   endif
#endif


namespace ariles
{
    /**
     * @brief YAML bridge namespace.
     */
    namespace yaml_cpp03
    {
    }
}
