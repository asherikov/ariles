/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
*/


#ifdef ARILES_ENABLED
#    define ARILES_TYPED_NAMED_ENTRY(v, type, entry, name) ARILES_NAMED_ENTRY(v, entry, name)

public:
// -----
// generic (templated) visitors
#    ifndef ARILES_DOXYGEN_PROCESSING
#        ifdef ARILES_ENTRIES
#            include "visit_generic_parent.h"
#            include "visit_generic_all.h"

// clang-format off
            ARILES_VISIT_generic_parent
            ARILES_VISIT_generic_all
// clang-format on

#        endif
#    endif
// -----


// -----
// Define node name
#    ifdef ARILES_DEFAULT_ID
        const std::string &
        arilesDefaultID() const
{
    static const std::string name(ARILES_DEFAULT_ID);
    return (name);
}
#    else
const std::string &arilesDefaultID() const
{
    static const std::string name("");
    return (name);
}
#    endif
// -----


// -----
// visitor-specific methods
#    ifndef ARILES_VISITORS
#        define ARILES_VISITORS ARILES_DEFAULT_VISITORS
#    endif

#    define ARILES_VISITOR(visitor) ARILES_METHODS_##visitor
ARILES_VISITORS
#    undef ARILES_VISITOR

#    ifdef ARILES_ENTRIES
#        define ARILES_VISITOR(visitor) ARILES_VISIT_##visitor
ARILES_VISITORS
#        undef ARILES_VISITOR
#    endif

#    undef ARILES_VISITORS
// -----


#    undef ARILES_TYPED_NAMED_ENTRY
#endif  // ARILES_ENABLED

#undef ARILES_DEFAULT_ID
