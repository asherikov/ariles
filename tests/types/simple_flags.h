/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableFlags0 : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "ConfigurableFlags0"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_TYPED_ENTRY_(real,        double)
        #define ARILES_AUTO_DEFAULTS
        #include ARILES_INITIALIZE


        public:
            ariles::ConfigurableFlags getExpectedConfigurableFlags() const
            {
                return (ariles::ConfigurableFlags(ariles::ConfigurableFlags::DEFAULT));
            }
    };


    class ConfigurableFlags1 : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "ConfigurableFlags1"
        #define ARILES_CONFIGURABLE_FLAGS   ariles::ConfigurableFlags::DEFAULT \
                                            | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED \
                                            | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_TYPED_ENTRY_(real,        double)
        #define ARILES_AUTO_DEFAULTS
        #include ARILES_INITIALIZE


        public:
            ariles::ConfigurableFlags getExpectedConfigurableFlags() const
            {
                return (ariles::ConfigurableFlags(
                            ariles::ConfigurableFlags::DEFAULT
                            | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                            | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED));
            }
    };


    class ConfigurableFlags2 : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "ConfigurableFlags2"
        #define ARILES_CONFIGURABLE_FLAGS   ariles::ConfigurableFlags::DEFAULT \
                                            & ~ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED \
                                            & ~ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_TYPED_ENTRY_(real,        double)
        #define ARILES_AUTO_DEFAULTS
        #include ARILES_INITIALIZE


        public:
            ariles::ConfigurableFlags getExpectedConfigurableFlags() const
            {
                return (ariles::ConfigurableFlags(
                            ariles::ConfigurableFlags::DEFAULT
                            & ~ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                            & ~ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED));
            }
    };
}
