/**
    @file
    @author Alexander Sherikov

    @copyright 2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Glue between ariles readers and the C interface of msgpack-c.
*/

#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>

#include <ariles2/visitors/msgpack.h>
#include <msgpack.h>


namespace ariles2
{
    namespace ns_msgpack_c
    {
        inline bool isEqual(const msgpack_object &object, const std::string &name)
        {
            return (MSGPACK_OBJECT_STR == object.type && name.size() == object.via.str.size
                    && 0 == std::char_traits<char>::compare(name.data(), object.via.str.ptr, name.size()));
        }


        template <class t_Type>
        void readUnsignedInteger(const msgpack_object &object, t_Type &value)
        {
            if (MSGPACK_OBJECT_POSITIVE_INTEGER != object.type)
            {
                CPPUT_THROW("Expected a positive integer.");
            }
            if (object.via.u64 > static_cast<std::uint64_t>(std::numeric_limits<t_Type>::max()))
            {
                CPPUT_THROW("Integer value is too large.");
            }
            value = static_cast<t_Type>(object.via.u64);
        }


        template <class t_Type>
        void readSignedInteger(const msgpack_object &object, t_Type &value)
        {
            if (MSGPACK_OBJECT_POSITIVE_INTEGER == object.type)
            {
                if (object.via.u64 > static_cast<std::uint64_t>(std::numeric_limits<t_Type>::max()))
                {
                    CPPUT_THROW("Integer value is too large.");
                }
                value = static_cast<t_Type>(object.via.u64);
            }
            else if (MSGPACK_OBJECT_NEGATIVE_INTEGER == object.type)
            {
                if (object.via.i64 < static_cast<std::int64_t>(std::numeric_limits<t_Type>::min()))
                {
                    CPPUT_THROW("Integer value is too small.");
                }
                value = static_cast<t_Type>(object.via.i64);
            }
            else
            {
                CPPUT_THROW("Expected an integer.");
            }
        }


        // msgpack-c has no overloaded interface, so the type has to be resolved here
        template <class t_Type>
        typename std::enable_if<std::is_integral<t_Type>::value && !std::is_same<t_Type, bool>::value>::type readValue(
                const msgpack_object &object,
                t_Type &value)
        {
            if constexpr (std::is_signed<t_Type>::value)
            {
                readSignedInteger(object, value);
            }
            else
            {
                readUnsignedInteger(object, value);
            }
        }


        inline void readValue(const msgpack_object &object, bool &value)
        {
            if (MSGPACK_OBJECT_BOOLEAN != object.type)
            {
                CPPUT_THROW("Expected a boolean.");
            }
            value = object.via.boolean;
        }


        inline void readValue(const msgpack_object &object, float &value)
        {
            if (MSGPACK_OBJECT_FLOAT32 == object.type || MSGPACK_OBJECT_FLOAT64 == object.type)
            {
                value = static_cast<float>(object.via.f64);
            }
            else if (MSGPACK_OBJECT_POSITIVE_INTEGER == object.type)
            {
                value = static_cast<float>(object.via.u64);
            }
            else if (MSGPACK_OBJECT_NEGATIVE_INTEGER == object.type)
            {
                value = static_cast<float>(object.via.i64);
            }
            else
            {
                CPPUT_THROW("Expected a floating point number.");
            }
        }


        inline void readValue(const msgpack_object &object, double &value)
        {
            if (MSGPACK_OBJECT_FLOAT32 == object.type || MSGPACK_OBJECT_FLOAT64 == object.type)
            {
                value = object.via.f64;
            }
            else if (MSGPACK_OBJECT_POSITIVE_INTEGER == object.type)
            {
                value = static_cast<double>(object.via.u64);
            }
            else if (MSGPACK_OBJECT_NEGATIVE_INTEGER == object.type)
            {
                value = static_cast<double>(object.via.i64);
            }
            else
            {
                CPPUT_THROW("Expected a floating point number.");
            }
        }


        inline void readValue(const msgpack_object &object, std::string &value)
        {
            if (MSGPACK_OBJECT_STR == object.type)
            {
                value.assign(object.via.str.ptr, object.via.str.size);
            }
            else if (MSGPACK_OBJECT_BIN == object.type)
            {
                value.assign(object.via.bin.ptr, object.via.bin.size);
            }
            else
            {
                CPPUT_THROW("Expected a string.");
            }
        }
    }  // namespace ns_msgpack_c
}  // namespace ariles2
