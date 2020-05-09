/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles/visitors/jsonnet.h>

extern "C"
{
#include "libjsonnet.h"
}

namespace ariles
{
    namespace ns_jsonnet
    {
        namespace impl
        {
            class ARILES_VISIBILITY_ATTRIBUTE JsonnetPreprocessor
            {
            public:
                JsonnetVm *vm_;
            };


            Reader::Reader()
            {
                preprocessor_ = JsonnetPreprocessorPtr(new JsonnetPreprocessor());
                preprocessor_->vm_ = static_cast<struct JsonnetVm *>(::jsonnet_make());
                ARILES_ASSERT(NULL != preprocessor_->vm_, "Could not initialize jsonnet preprocessor.");
            }


            Reader::~Reader()
            {
                ::jsonnet_destroy(preprocessor_->vm_);
            }


            const char *Reader::fromFile(const std::string &file_name)
            {
                int error = 0;
                const char *jsonnet_output = ::jsonnet_evaluate_file(preprocessor_->vm_, file_name.c_str(), &error);
                ARILES_ASSERT(0 == error, jsonnet_output);
                return (jsonnet_output);
            }


            const char *Reader::fromString(const std::string &input_string)
            {
                int error = 0;
                const char *jsonnet_output =
                        ::jsonnet_evaluate_snippet(preprocessor_->vm_, "<input steam>", input_string.c_str(), &error);
                ARILES_ASSERT(0 == error, jsonnet_output);
                return (jsonnet_output);
            }
        }  // namespace impl
    }      // namespace ns_jsonnet
}  // namespace ariles
