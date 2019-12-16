/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    class ConfigurableFlags : public ariles::Flags<unsigned int, ConfigurableFlags>
    {
        public:
            enum Flags
            {
                RESET = 0,
                ALLOW_MISSING_ENTRIES = 1,
                SLOPPY_MAPS_IF_SUPPORTED = 2,
                COMPACT_ARRAYS_IF_SUPPORTED = 4,
                PROPAGATE_ALLOW_MISSING_ENTRIES = 8,
                FORCE_EXPLICIT_MATRIX_SIZE = 16,
                SLOPPY_PAIRS_IF_SUPPORTED = 32,

#ifdef ARILES_DEFAULT_CONFIGURABLE_FLAGS
                DEFAULT = ARILES_DEFAULT_CONFIGURABLE_FLAGS
#else
                DEFAULT = RESET
#endif
            };


        public:
            ConfigurableFlags()
            {
                setDefaults();
            }


            ConfigurableFlags(const unsigned int flags, const Action action_type = REPLACE)
            {
                initialize(flags, action_type);
            }


            void setDefaults()
            {
                flags_ = DEFAULT;
            }
    };
}
