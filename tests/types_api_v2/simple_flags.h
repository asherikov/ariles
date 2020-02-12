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
    class ConfigurableFlags1 : public ariles::Base
    {
        #define ARILES_DEFAULT_ID "ConfigurableFlags1"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_TYPED_ENTRY_(real,        double)
        #include ARILES_INITIALIZE


        public:
            ariles::ConfigurableFlags getExpectedConfigurableFlags() const
            {
                return (ariles::ConfigurableFlags(
                            ariles::ConfigurableFlags::DEFAULT
                            | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                            | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED));
            }

            virtual const ariles::read::Visitor::Parameters &arilesGetParameters(const ariles::read::Visitor &) const
            {
                static ariles::read::Visitor::Parameters flags(
                            ariles::ConfigurableFlags::DEFAULT
                            | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                            | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
                return (flags);
            }

            virtual const ariles::write::Visitor::Parameters &arilesGetParameters(const ariles::write::Visitor &) const
            {
                static ariles::write::Visitor::Parameters flags(
                            ariles::ConfigurableFlags::DEFAULT
                            | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                            | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
                return (flags);
            }
    };


    class ConfigurableFlags2 : public ariles::Base
    {
        #define ARILES_DEFAULT_ID "ConfigurableFlags2"
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

            virtual const ariles::read::Visitor::Parameters &arilesGetParameters(const ariles::read::Visitor &) const
            {
                static ariles::read::Visitor::Parameters flags(
                            ariles::ConfigurableFlags::DEFAULT
                            & ~ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                            & ~ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
                return (flags);
            }

            virtual const ariles::write::Visitor::Parameters &arilesGetParameters(const ariles::write::Visitor &) const
            {
                static ariles::write::Visitor::Parameters flags(
                            ariles::ConfigurableFlags::DEFAULT
                            & ~ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                            & ~ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
                return (flags);
            }
    };
}
