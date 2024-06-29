/**
    @file
    @author  Alexander Sherikov
    @copyright 2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#ifndef H_CPPUT_CONCAT
#define H_CPPUT_CONCAT

#include <string>
#include <string_view>

namespace cpput
{
    namespace concat
    {
        template <typename... t_String>
        std::string reserve_strings(const t_String &...strings)
        {
            std::string result;
            result.reserve((strings.size() + ...));
            (result += ... += strings);
            return (result);
        }


        template <typename... t_String>
        std::string reserve(t_String &&...strings)
        {
            return (reserve_strings(std::string_view(strings)...));
        }


        template <typename... t_String>
        std::string simple(t_String &&...strings)
        {
            std::string result;
            (result += ... += strings);
            return result;
        }
    }  // namespace concat
}  // namespace cpput

#endif
