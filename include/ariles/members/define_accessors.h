/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Inclusion of this file results in generation of functions which
    read and write entries 'ARILES_ENTRIES' defined in the including
    header from / to a configuration file.
*/


#ifdef ARILES_SECTION_ID
    #define ARILES_DEFAULT_ID ARILES_SECTION_ID
#endif

#include "variables.h"
#include "methods.h"
#include "methods_deprecated.h"
