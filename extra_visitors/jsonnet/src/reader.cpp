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
            class ARILES2_VISIBILITY_ATTRIBUTE JsonnetPreprocessor
            {
            public:
                JsonnetVm *vm_;
            };


            Reader::Reader()
            {
                preprocessor_ = std::make_shared<JsonnetPreprocessor>();
                preprocessor_->vm_ = static_cast<struct JsonnetVm *>(::jsonnet_make());
                ARILES2_ASSERT(NULL != preprocessor_->vm_, "Could not initialize jsonnet preprocessor.");
            }


            Reader::~Reader()
            {
                ::jsonnet_destroy(preprocessor_->vm_);
            }


            const char *Reader::fromFile(const std::string &file_name)
            {
                int error = 0;
                const char *jsonnet_output = ::jsonnet_evaluate_file(preprocessor_->vm_, file_name.c_str(), &error);
                ARILES2_ASSERT(0 == error, jsonnet_output);
                return (jsonnet_output);
            }


            const char *Reader::fromString(const std::string &input_string)
            {
                int error = 0;
                const char *jsonnet_output =
                        ::jsonnet_evaluate_snippet(preprocessor_->vm_, "<input steam>", input_string.c_str(), &error);
                ARILES2_ASSERT(0 == error, jsonnet_output);
                return (jsonnet_output);
            }
        }  // namespace impl
    }      // namespace ns_jsonnet
}  // namespace ariles2
