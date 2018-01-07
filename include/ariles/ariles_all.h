/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include "internal/build_config.h"

#ifdef ARILES_BRIDGE_msgpack
#   include "format_msgpack.h"
#endif

#ifdef ARILES_BRIDGE_yaml
#   include "format_yaml.h"
#endif

#include "adapters_all.h"
#include "ariles.h"
