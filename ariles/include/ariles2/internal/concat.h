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
#include <type_traits>

namespace cpput
{
    namespace concat
    {
        template <typename... t_String>
        std::string reserve_strings(const t_String &...strings)
        {
            std::string result;
            result.reserve((0 + ... + strings.size()));
            static_cast<void>((result += ... += strings));
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
            static_assert(
                    (std::is_convertible_v<t_String, std::string_view> && ...)
                            || (std::is_convertible_v<t_String, std::string> && ...),
                    "cpput::concat::simple() only accepts strings.");
            std::string result;
            static_cast<void>((result += ... += strings));
            return result;
        }
    }  // namespace concat
}  // namespace cpput

#endif
