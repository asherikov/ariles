/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "reader.h"

namespace ariles
{
    namespace bridge
    {
        namespace rapidjson
        {
            namespace jsonnet
            {
                class ARILES_LIB_EXPORT Reader : public rapidjson::Reader
                {
                    public:
                        explicit Reader(const std::string& file_name);
                        explicit Reader(std::istream & input_stream);
                };
            }
        }
    }
}
