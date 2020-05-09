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


#ifndef ARILES_DOXYGEN_PROCESSING
public:
#    ifdef ARILES_ENTRIES
#        define ARILES_TYPED_NAMED_ENTRY(v, type, entry, name) ARILES_TYPED_NAMED_ENTRY_##v(v, type, entry, name)

#        define ARILES_NAMED_ENTRY_members(v, entry, name)
#        define ARILES_PARENT_members(v, entry)
#        define ARILES_TYPED_NAMED_ENTRY_members(v, type, entry, name) type entry;

ARILES_ENTRIES(members)

#        undef ARILES_TYPED_NAMED_ENTRY
#    endif
#endif
