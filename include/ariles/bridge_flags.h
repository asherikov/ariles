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
    class BridgeFlags : public Flags<BridgeFlags>
    {
        public:
            enum Flags
            {
                RESET = 0,
                SLOPPY_MAPS_SUPPORTED = 1,
                NATIVE_MATRIX_SUPPORTED = 2
            };


        public:
            BridgeFlags()
            {
                setDefaults();
            }

            BridgeFlags(const uint64_t flags, const Action action_type = SET)
            {
                initialize(flags, action_type);
            }


            void setDefaults()
            {
                flags_ = RESET;

                // off
                unset(SLOPPY_MAPS_SUPPORTED | NATIVE_MATRIX_SUPPORTED);
            }
    };
}
