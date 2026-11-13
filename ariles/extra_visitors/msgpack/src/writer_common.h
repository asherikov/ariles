/**
    @file
    @author Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Glue between ariles writers and the C interface of msgpack-c.
*/

#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <type_traits>

#include <ariles2/visitors/msgpack.h>
#include <msgpack.h>


namespace ariles2
{
    namespace ns_msgpack_c
    {
        inline int writeToStream(void *data, const char *buffer, const std::size_t length)
        {
            static_cast<std::ostream *>(data)->write(buffer, static_cast<std::streamsize>(length));
            return 0;
        }


        inline void packValue(msgpack_packer *packer, const bool value)
        {
            if (value)
            {
                msgpack_pack_true(packer);
            }
            else
            {
                msgpack_pack_false(packer);
            }
        }


        // msgpack-c has no overloaded interface, so the type has to be resolved here
        template <class t_Type>
        typename std::enable_if<std::is_integral<t_Type>::value && !std::is_same<t_Type, bool>::value>::type packValue(
                msgpack_packer *packer,
                const t_Type &value)
        {
            if constexpr (std::is_signed<t_Type>::value)
            {
                msgpack_pack_int64(packer, static_cast<std::int64_t>(value));
            }
            else
            {
                msgpack_pack_uint64(packer, static_cast<std::uint64_t>(value));
            }
        }


        inline void packValue(msgpack_packer *packer, const float value)
        {
            msgpack_pack_float(packer, value);
        }


        inline void packValue(msgpack_packer *packer, const double value)
        {
            msgpack_pack_double(packer, value);
        }


        inline void packValue(msgpack_packer *packer, const std::string &value)
        {
            // msgpack_pack_str_with_body is not available in older msgpack-c versions
            msgpack_pack_str(packer, value.size());
            msgpack_pack_str_body(packer, value.data(), value.size());
        }
    }  // namespace ns_msgpack_c
}  // namespace ariles2
