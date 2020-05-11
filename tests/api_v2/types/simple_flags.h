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
    class ConfigurableFlags1 : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE


    public:
        ariles2::ConfigurableFlags getExpectedConfigurableFlags() const
        {
            return (ariles2::ConfigurableFlags(
                    ariles2::ConfigurableFlags::DEFAULT | ariles2::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    | ariles2::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED));
        }

        virtual const ariles2::Read::Parameters &arilesGetParameters(const ariles2::Read &) const
        {
            static ariles2::Read::Parameters flags(
                    ariles2::ConfigurableFlags::DEFAULT | ariles2::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    | ariles2::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
            return (flags);
        }

        virtual const ariles2::Write::Parameters &arilesGetParameters(const ariles2::Write &) const
        {
            static ariles2::Write::Parameters flags(
                    ariles2::ConfigurableFlags::DEFAULT | ariles2::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    | ariles2::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
            return (flags);
        }
    };


    class ConfigurableFlags2 : public ariles2::DefaultBase
    {
#define ARILES2_DEFAULT_ID "ConfigurableFlags2"
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
#define ARILES_AUTO_DEFAULTS
#include ARILES2_INITIALIZE


    public:
        ariles2::ConfigurableFlags getExpectedConfigurableFlags() const
        {
            return (ariles2::ConfigurableFlags(
                    ariles2::ConfigurableFlags::DEFAULT & ~ariles2::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    & ~ariles2::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED));
        }

        virtual const ariles2::Read::Parameters &arilesGetParameters(const ariles2::Read &) const
        {
            static ariles2::Read::Parameters flags(
                    ariles2::ConfigurableFlags::DEFAULT & ~ariles2::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    & ~ariles2::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
            return (flags);
        }

        virtual const ariles2::Write::Parameters &arilesGetParameters(const ariles2::Write &) const
        {
            static ariles2::Write::Parameters flags(
                    ariles2::ConfigurableFlags::DEFAULT & ~ariles2::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED
                    & ~ariles2::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED);
            return (flags);
        }
    };
}  // namespace ariles_tests
