/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_INCLUDED_ADAPTER_BOOST_POINTER


#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared.hpp>

#define ARILES_POINTER_TYPE(entry_type) boost::shared_ptr<entry_type>
#include <ariles/adapters/generic_pointer.h>


#include <boost/move/unique_ptr.hpp>
#include <boost/move/make_unique.hpp>

#define ARILES_POINTER_TYPE(entry_type) boost::movelib::unique_ptr<entry_type>
#include <ariles/adapters/generic_pointer.h>
