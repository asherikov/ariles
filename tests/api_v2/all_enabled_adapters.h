/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <ariles2/adapters/std_vector.h>
#include <ariles2/adapters/std_pair.h>
#include <ariles2/adapters/std_map.h>
#include <ariles2/adapters/pointer.h>

#ifdef ARILES_ADAPTER_EIGEN
#    include <ariles2/adapters/eigen.h>
#endif
#ifdef ARILES_ADAPTER_BOOST_POINTER
#    include <ariles2/adapters/boost_pointer.h>
#endif
#ifdef ARILES_ADAPTER_BOOST_OPTIONAL
#    include <ariles2/adapters/boost_optional.h>
#endif
#ifdef ARILES_ADAPTER_BETTER_ENUMS
#    include <ariles2/adapters/better_enums.h>
#endif
