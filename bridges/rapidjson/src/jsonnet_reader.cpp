/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#define ARILES_BRIDGE_INCLUDED_jsonnet
#include <ariles/visitors/rapidjson.h>

extern "C" {
#include "libjsonnet.h"
}

namespace ariles
{
    namespace bridge
    {
        namespace rapidjson
        {
            namespace jsonnet
            {
                Reader::Reader(const std::string& file_name, const Flags &flags) : rapidjson::Reader(flags)
                {
                    struct JsonnetVm* vm = static_cast<struct JsonnetVm*>(::jsonnet_make());
                    ARILES_ASSERT(NULL != vm, "Could not initialize jsonnet preprocessor.");

                    int error = 0;
                    const char* jsonnet_output = ::jsonnet_evaluate_file(vm, file_name.c_str(), &error);
                    ARILES_ASSERT(0 == error, jsonnet_output);

                    constructFromString(jsonnet_output);

                    ::jsonnet_destroy(vm);
                }


                Reader::Reader(std::istream & input_stream, const Flags &flags) : rapidjson::Reader(flags)
                {
                    std::string input_string;
                    char buffer[4096];
                    while (input_stream.read(buffer, sizeof(buffer)))
                    {
                        input_string.append(buffer, sizeof(buffer));
                    }
                    input_string.append(buffer, input_stream.gcount());


                    struct JsonnetVm* vm = static_cast<struct JsonnetVm*>(::jsonnet_make());
                    ARILES_ASSERT(NULL != vm, "Could not initialize jsonnet preprocessor.");


                    int error = 0;
                    const char* jsonnet_output =
                        ::jsonnet_evaluate_snippet(vm, "<input steam>", input_string.c_str(), &error);
                    ARILES_ASSERT(0 == error, jsonnet_output);


                    constructFromString(jsonnet_output);

                    ::jsonnet_destroy(vm);
                }
            }
        }
    }
}
