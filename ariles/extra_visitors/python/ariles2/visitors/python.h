/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup python Python
@ingroup config

@brief Python serialization.

@brief Outputs a Python script that defines variables containing data exported by ariles.
Script includes all necessary imports and is intended to be loaded in other python scripts
or interactive sessions using `exec(open("output_script.py").read())`. Ariles class exported
using 'python' visitor is a single python map similar to this:
`root_element={'vector': [1, 2, 3], 'vector_of_maps': [{'a': 'a'}], 'scalar': 2}`,
i.e.: ariles maps correspond to python dictionaries, arrays to python arrays, matrices and
vectors should be represented using numpy types.

@note This visitor only supports writing (serialization) as reading Python files properly
requires a full Python parser which is beyond the scope of this library.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_python

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>


#include "./python/writer.h"


namespace ariles2
{
    /**
     * @brief Python visitor.
     * @ingroup python
     */
    struct python
    {
        using WriterBase = ns_python::Writer;

        using Writer = ariles2::cfgwrite::Visitor<ns_python::Writer>;

        // Python visitor is write-only; reading Python files requires a full Python parser
        // The Reader type is defined but not implemented for interface compatibility
    };
}  // namespace ariles2