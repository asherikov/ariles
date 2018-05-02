/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_INCLUDED_ADAPTER_BOOST_OPTIONAL


#include <boost/optional.hpp>

#define ARILES_POINTER_TYPE(entry_type) boost::optional<entry_type>
#include <ariles/adapters/generic_pointer.h>
