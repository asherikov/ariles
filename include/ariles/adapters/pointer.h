/**
    @file
    @author  Alexander Sherikov
    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#define ARILES_INCLUDED_ADAPTER_POINTER


#if __cplusplus >= 201103L

#include <memory>
#define ARILES_POINTER_TYPE(entry_type) std::shared_ptr<entry_type>
#include <ariles/adapters/generic_pointer.h>

#define ARILES_POINTER_TYPE(entry_type) std::unique_ptr<entry_type>
#include <ariles/adapters/generic_pointer.h>

#endif
