/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/
// cppcheck-suppress-file duplInheritedMember

#pragma once


namespace ariles_tests
{
    class ConfigurablePlain : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE

    public:
        bool defaults_visited_;
        bool finalize_visited_;
        bool prewrite_visited_;


    public:
        ConfigurablePlain()
          : integer_(0), real_(0.0), defaults_visited_(false), finalize_visited_(false), prewrite_visited_(false)
        {
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            defaults_visited_ = true;
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            finalize_visited_ = true;
        }

        void arilesVisit(const ariles2::PreWrite & /*visitor*/, const ariles2::PreWrite::Parameters & /*param*/)
        {
            prewrite_visited_ = true;
        }
    };
}  // namespace ariles_tests
