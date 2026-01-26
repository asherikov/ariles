/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/jsonnet.h>

extern "C"
{
#include "libjsonnet.h"
}

namespace ariles2
{
    namespace ns_jsonnet
    {
        namespace impl
        {
            class JsonnetPreprocessor
            {
            public:
                JsonnetVm *vm_;
            };


            Reader::Reader()
            {
                preprocessor_ = std::make_shared<JsonnetPreprocessor>();
                preprocessor_->vm_ = ::jsonnet_make();
                CPPUT_ASSERT(nullptr != preprocessor_->vm_, "Could not initialize jsonnet preprocessor.");
            }


            Reader::~Reader()
            {
                ::jsonnet_destroy(preprocessor_->vm_);
            }


            char *Reader::fromFile(const std::string &file_name)
            {
                int error = 0;
                char *jsonnet_output = ::jsonnet_evaluate_file(preprocessor_->vm_, file_name.c_str(), &error);
                CPPUT_ASSERT(0 == error, jsonnet_output);
                return (jsonnet_output);
            }


            char *Reader::fromString(const std::string &input_string)
            {
                int error = 0;
                char *jsonnet_output =
                        ::jsonnet_evaluate_snippet(preprocessor_->vm_, "<input steam>", input_string.c_str(), &error);
                CPPUT_ASSERT(0 == error, jsonnet_output);
                return (jsonnet_output);
            }

            void Reader::free(char *jsonnet_data)
            {
                cpput::ignoreResult(jsonnet_realloc(preprocessor_->vm_, jsonnet_data, 0));
            }
        }  // namespace impl
    }  // namespace ns_jsonnet
}  // namespace ariles2
