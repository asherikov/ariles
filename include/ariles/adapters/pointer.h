/**
    @file
    @author  Alexander Sherikov
    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once
#if __cplusplus >= 201103L

#include <memory>

#define ARILES_POINTER_TYPE(entry_type)                 std::shared_ptr<entry_type>
#define ARILES_POINTER_ALLOCATE(entry_type, pointer)    pointer = std::make_shared<entry_type>()
#define ARILES_POINTER_RESET(pointer)                   pointer.reset()
#define ARILES_POINTER_CHECK_DEFINED(pointer)           (NULL == pointer)
#include <ariles/adapters/generic_pointer.h>


#define ARILES_POINTER_TYPE(entry_type)                 std::unique_ptr<entry_type>
#define ARILES_POINTER_ALLOCATE(entry_type, pointer)    pointer.reset(new double())
#define ARILES_POINTER_RESET(pointer)                   pointer.reset(NULL)
#define ARILES_POINTER_CHECK_DEFINED(pointer)           (NULL == pointer)
#include <ariles/adapters/generic_pointer.h>

#endif
