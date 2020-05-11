/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <ariles2/visitors/pugixml.h>
#include <boost/lexical_cast.hpp>
#include <pugixml.hpp>

namespace ariles
{
    namespace ns_pugixml
    {
        typedef ariles::Node<pugi::xml_node> NodeWrapper;
    }
}  // namespace ariles
