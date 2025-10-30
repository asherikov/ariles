/**
    @file
    @author  Alexander Sherikov

    @copyright 2025 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/
// cppcheck-suppress-file duplInheritedMember

#pragma once

#include <ariles2/extra.h>

namespace ariles_tests
{
    class ConfigurableIteratorIncrementSegfault : public ariles2::RelaxedSloppyBase
    {
        using MapMemberType = std::map<std::string, int>;
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, map_member, MapMemberType)
#include ARILES2_INITIALIZE


    public:
#ifndef ARILES_TESTS_RANDOMIZE_DISABLED
        void randomize()
        {
            map_member_["a"] = 1;
            map_member_["b"] = 2;
            map_member_["c"] = 2;
        }
#endif
    };
}  // namespace ariles_tests
