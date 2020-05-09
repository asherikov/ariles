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
    class ConfigurableFlags1 : public ariles::DefaultBase
    {
#define ARILES_ENTRIES(v)                                                                                              \
    ARILES_TYPED_ENTRY_(v, integer, int)                                                                               \
    ARILES_TYPED_ENTRY_(v, real, double)
#include ARILES_INITIALIZE


    public:
        ariles::ConfigurableFlags getExpectedConfigurableFlags() const
        {
            return (ariles::ConfigurableFlags(
                    ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED));
        }

        virtual const ariles::Read::Parameters &arilesGetParameters(const ariles::Read &) const
        {
            static ariles::Read::Parameters flags(
                    ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
            return (flags);
        }

        virtual const ariles::Write::Parameters &arilesGetParameters(const ariles::Write &) const
        {
            static ariles::Write::Parameters flags(
                    ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
            return (flags);
        }
    };


    class ConfigurableFlags2 : public ariles::DefaultBase
    {
#define ARILES_DEFAULT_ID "ConfigurableFlags2"
#define ARILES_ENTRIES(v)                                                                                              \
    ARILES_TYPED_ENTRY_(v, integer, int)                                                                               \
    ARILES_TYPED_ENTRY_(v, real, double)
#define ARILES_AUTO_DEFAULTS
#include ARILES_INITIALIZE


    public:
        ariles::ConfigurableFlags getExpectedConfigurableFlags() const
        {
            return (ariles::ConfigurableFlags(
                    ariles::ConfigurableFlags::DEFAULT & ~ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    & ~ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED));
        }

        virtual const ariles::Read::Parameters &arilesGetParameters(const ariles::Read &) const
        {
            static ariles::Read::Parameters flags(
                    ariles::ConfigurableFlags::DEFAULT & ~ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    & ~ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
            return (flags);
        }

        virtual const ariles::Write::Parameters &arilesGetParameters(const ariles::Write &) const
        {
            static ariles::Write::Parameters flags(
                    ariles::ConfigurableFlags::DEFAULT & ~ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    & ~ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
            return (flags);
        }
    };
}  // namespace ariles_tests
