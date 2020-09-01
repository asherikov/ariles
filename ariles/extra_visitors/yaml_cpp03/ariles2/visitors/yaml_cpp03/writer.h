/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles2
{
    namespace ns_yaml_cpp03
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



            void startMap(const Parameters &, const std::size_t /*num_entries*/);
            void startMapEntry(const std::string &map_name);
            void endMap();


            void flush();


            void startArray(const std::size_t /*size*/, const bool compact = false);
            void endArray();


            void startRoot(const std::string &name, const Parameters &);
            void endRoot(const std::string &name);


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element, const Parameters &param);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_yaml_cpp03
}  // namespace ariles2
