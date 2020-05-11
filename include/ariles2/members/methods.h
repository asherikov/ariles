/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
*/


#ifdef ARILES2_ENABLED
#    define ARILES2_TYPED_NAMED_ENTRY(v, type, entry, name) ARILES2_NAMED_ENTRY(v, entry, name)

public:
// -----
// generic (templated) visitors
#    ifndef ARILES_DOXYGEN_PROCESSING
#        ifdef ARILES2_ENTRIES
#            include "visit_generic_parent.h"
#            include "visit_generic_all.h"

// clang-format off
            ARILES2_VISIT_generic_parent
            ARILES2_VISIT_generic_all
// clang-format on

#        endif
#    endif
// -----


// -----
// Define node name
#    ifdef ARILES2_DEFAULT_ID
        const std::string &
        arilesDefaultID() const
{
    static const std::string name(ARILES2_DEFAULT_ID);
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
#    ifndef ARILES2_VISITORS
#        define ARILES2_VISITORS ARILES2_DEFAULT_VISITORS
#    endif

#    define ARILES2_VISITOR(visitor) ARILES2_METHODS_##visitor
ARILES2_VISITORS
#    undef ARILES2_VISITOR

#    ifdef ARILES2_ENTRIES
#        define ARILES2_VISITOR(visitor) ARILES2_VISIT_##visitor
ARILES2_VISITORS
#        undef ARILES2_VISITOR
#    endif

#    undef ARILES2_VISITORS
// -----


#    undef ARILES2_TYPED_NAMED_ENTRY
#endif  // ARILES2_ENABLED

#undef ARILES2_DEFAULT_ID
