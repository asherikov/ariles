/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles2
{
    namespace ns_python
    {
        namespace impl
        {
            class Writer;
        }


        /**
         * @brief Configuration writer class
         */
        class Writer : public serialization::PIMPLVisitor<write::Visitor, impl::Writer>
        {
        public:
            explicit Writer(const std::string &file_name);
            explicit Writer(std::ostream &output_stream);

            void flush();

            void startRoot(const std::string &name, const Parameters &);

            void startMap(const Parameters &, const std::size_t /*num_entries*/);
            void endMap();
            void startMapEntry(const std::string &map_name);
            void endMapEntry();


            void startVector(const std::size_t size);
            void endVectorElement();
            void endVector();

            void startMatrix(
                    const bool dynamic,
                    const std::size_t cols,
                    const std::size_t rows,
                    const Parameters &param);
            void startMatrixRow(const std::size_t cols, const Parameters &param);
            void endMatrixElement();
            void endMatrixRow(const Parameters &param);
            void endMatrix(const bool dynamic, const Parameters &param);

            void startArray(const std::size_t size, const bool /*compact*/ = false);
            void endArrayElement();
            void endArray();


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element, const Parameters &param);

            CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_python
}  // namespace ariles2
