/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles2
{
    namespace ns_rapidjson
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer;
        }


        /**
         * @brief Configuration writer class
         */
        class ARILES2_VISIBILITY_ATTRIBUTE Writer : public ns_rapidjson::Base<ariles2::write::Visitor, impl::Writer>
        {
        public:
            explicit Writer(const std::string &file_name, const Flags &flags = Flags::DEFAULT);
            explicit Writer(std::ostream &output_stream, const Flags &flags = Flags::DEFAULT);

            void flush();


            void descend(const std::string &map_name);
            void ascend();

            void startMap(const std::string & /*id*/, const std::size_t /*num_entries*/);
            void startArray(const std::size_t size, const bool /*compact*/ = false);

            void startArrayElement();
            void endArrayElement();
            void endArray();



#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_rapidjson
}  // namespace ariles2
