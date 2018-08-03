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
    class ConfigurableParameters
    {
        public:
            bool crash_on_missing_entry_;
            bool enable_sloppy_maps_if_supported_;
            bool compact_arrays_if_supported_;
            bool override_crash_on_missing_entry_;
            bool force_explicit_matrix_size_;

        public:
            ConfigurableParameters(const bool crash_on_missing_entry)
            {
                crash_on_missing_entry_ = crash_on_missing_entry;
#ifdef ARILES_ENABLE_SLOPPY_MAP
                enable_sloppy_maps_if_supported_ = true;
#else
                enable_sloppy_maps_if_supported_ = false;
#endif
                compact_arrays_if_supported_ = false;
                override_crash_on_missing_entry_ = false;

                force_explicit_matrix_size_ = false;
            }
    };
}
