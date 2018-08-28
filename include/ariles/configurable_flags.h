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
    class ConfigurableFlags : public Flags<ConfigurableFlags>
    {
        public:
            enum Flags
            {
                RESET = 0,
                CRASH_ON_MISSING_ENTRY = 1,
                SLOPPY_MAPS_IF_SUPPORTED = 2,
                COMPACT_ARRAYS_IF_SUPPORTED = 4,
                OVERRIDE_CRASH_ON_MISSING_ENTRY = 8,
                FORCE_EXPLICIT_MATRIX_SIZE = 16
            };


        public:
            ConfigurableFlags()
            {
                setDefaults();
            }


            /// @todo delete
            ConfigurableFlags(const bool strict);

            ConfigurableFlags(const Flags flags, const Action action_type = SET)
            {
                initialize(flags, action_type);
            }


            void setDefaults()
            {
#ifdef ARILES_DEFAULT_CONFIGURABLE_FLAGS
                flags_ = ARILES_DEFAULT_CONFIGURABLE_FLAGS;
#else
                flags_ = RESET;

                // on
                set(CRASH_ON_MISSING_ENTRY);


                // conditional
#   ifdef ARILES_ENABLE_SLOPPY_MAP
                set(SLOPPY_MAPS_IF_SUPPORTED);
#   else
                unset(SLOPPY_MAPS_IF_SUPPORTED);
#   endif

                // off
                unset(COMPACT_ARRAYS_IF_SUPPORTED
                        | OVERRIDE_CRASH_ON_MISSING_ENTRY
                        | FORCE_EXPLICIT_MATRIX_SIZE);
#endif
            }
    };
}
