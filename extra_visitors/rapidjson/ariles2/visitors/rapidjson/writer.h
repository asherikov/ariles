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
        class ARILES2_VISIBILITY_ATTRIBUTE Writer : public serialization::PIMPLVisitor<write::Visitor, impl::Writer>
        {
        public:
            explicit Writer(const std::string &file_name);
            explicit Writer(std::ostream &output_stream);

            void flush();


            void startMap(const Parameters &, const std::size_t /*num_entries*/);
            void startMapEntry(const std::string &map_name);
            void endMapEntry();


            void startArray(const std::size_t size, const bool /*compact*/ = false);
            void startArrayElement();
            void endArrayElement();
            void endArray();


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element, const Parameters &param);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_rapidjson
}  // namespace ariles2
