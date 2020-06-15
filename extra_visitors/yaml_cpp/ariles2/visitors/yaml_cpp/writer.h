/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles2
{
    namespace ns_yaml_cpp
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer;
        }


        /**
         * @brief Configuration reader class
         */
        class ARILES2_VISIBILITY_ATTRIBUTE Writer : public serialization::PIMPLVisitor<write::Visitor, impl::Writer>
        {
        public:
            explicit Writer(const std::string &file_name);
            explicit Writer(std::ostream &output_stream);


            void startMap(const std::string & /*id*/, const std::size_t /*num_entries*/);
            void startMapElement(const std::string &map_name);
            void endMap();


            void flush();


            void startArray(const std::size_t /*size*/, const bool compact = false);
            void endArray();

            void startRoot(const std::string &name);
            void endRoot(const std::string &name);


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element, const Parameters &param);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_yaml_cpp
}  // namespace ariles2
