/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

// ARILES_POINTER_TYPE(entry_type)

namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader &reader,
                        ARILES_POINTER_TYPE(t_Entry) &entry,
                        const ariles::ConfigurableFlags & param);
    }


    namespace writer
    {
        template <  class t_Writer,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const ARILES_POINTER_TYPE(t_Entry) &entry,
                        const ariles::ConfigurableFlags & param);
    }
}

#undef ARILES_POINTER_TYPE
