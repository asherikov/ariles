/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    namespace ns_jsonnet
    {
        namespace impl
        {
            class ARILES_VISIBILITY_ATTRIBUTE JsonnetPreprocessor;

            class ARILES_VISIBILITY_ATTRIBUTE Reader
            {
            protected:
                typedef ARILES_SHARED_PTR<JsonnetPreprocessor> JsonnetPreprocessorPtr;

            protected:
                JsonnetPreprocessorPtr preprocessor_;


            public:
                Reader();
                ~Reader();


                const char *fromFile(const std::string &file_name);
                const char *fromString(const std::string &input_string);
            };
        }  // namespace impl


        template <class t_ParentVisitor>
        class ARILES_VISIBILITY_ATTRIBUTE Reader : public t_ParentVisitor
        {
        protected:
            impl::Reader impl_;


        public:
            explicit Reader(const std::string &file_name) : t_ParentVisitor()
            {
                const char *jsonnet_output = impl_.fromFile(file_name);
                t_ParentVisitor::constructFromString(jsonnet_output);
            }

            template <class t_Flags>
            explicit Reader(const std::string &file_name, const t_Flags &flags) : t_ParentVisitor(flags)
            {
                const char *jsonnet_output = impl_.fromFile(file_name);
                t_ParentVisitor::constructFromString(jsonnet_output);
            }


            explicit Reader(std::istream &input_stream) : t_ParentVisitor()
            {
                std::string input_string;
                char buffer[4096];
                while (input_stream.read(buffer, sizeof(buffer)))
                {
                    input_string.append(buffer, sizeof(buffer));
                }
                input_string.append(buffer, input_stream.gcount());


                const char *jsonnet_output = impl_.fromString(input_string);
                t_ParentVisitor::constructFromString(jsonnet_output);
            }

            template <class t_Flags>
            explicit Reader(std::istream &input_stream, const t_Flags &flags) : t_ParentVisitor(flags)
            {
                std::string input_string;
                char buffer[4096];
                while (input_stream.read(buffer, sizeof(buffer)))
                {
                    input_string.append(buffer, sizeof(buffer));
                }
                input_string.append(buffer, input_stream.gcount());


                const char *jsonnet_output = impl_.fromString(input_string);
                t_ParentVisitor::constructFromString(jsonnet_output);
            }
        };
    }  // namespace ns_jsonnet
}  // namespace ariles
