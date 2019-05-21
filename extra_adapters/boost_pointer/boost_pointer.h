/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared.hpp>

#define ARILES_POINTER_TYPE(entry_type)                 boost::shared_ptr<entry_type>
#define ARILES_POINTER_ALLOCATE(entry_type, pointer)    pointer = boost::make_shared<entry_type>()
#define ARILES_POINTER_RESET(pointer)                   pointer.reset()
#define ARILES_POINTER_CHECK_NULL(pointer)              (NULL == pointer)
#include <ariles/adapters/generic_pointer.h>


// this version is known to work
#if BOOST_VERSION >= 105800
#   include <boost/move/unique_ptr.hpp>
#   include <boost/move/make_unique.hpp>

#   define ARILES_POINTER_TYPE(entry_type)                 boost::movelib::unique_ptr<entry_type>
#   define ARILES_POINTER_ALLOCATE(entry_type, pointer)    pointer = boost::movelib::make_unique<entry_type>()
#   define ARILES_POINTER_RESET(pointer)                   pointer.reset()
#   define ARILES_POINTER_CHECK_NULL(pointer)              (NULL == pointer)
#   include <ariles/adapters/generic_pointer.h>
#endif
