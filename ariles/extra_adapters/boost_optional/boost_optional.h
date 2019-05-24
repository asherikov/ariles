/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/optional.hpp>


#define ARILES_POINTER_TYPE(entry_type)                 boost::optional<entry_type>
#define ARILES_POINTER_ALLOCATE(entry_type, pointer)    pointer = entry_type()
#define ARILES_POINTER_RESET(pointer)                   pointer = boost::none
#define ARILES_POINTER_CHECK_NULL(pointer)              (boost::none == pointer)
#include <ariles/adapters/generic_pointer.h>
