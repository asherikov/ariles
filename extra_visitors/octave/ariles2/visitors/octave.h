/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES2_VISITOR_INCLUDED_octave

#include <ariles2/internal/helpers.h>
#include <ariles2/internal/node.h>
#include <ariles2/visitors/config.h>


namespace ariles2
{
    namespace ns_octave
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer;
        }


        /**
         * @brief Configuration writer class
         */
        class ARILES2_VISIBILITY_ATTRIBUTE Writer : public ariles2::write::Visitor
        {
        protected:
            typedef impl::Writer Impl;
            typedef ARILES2_SHARED_PTR<impl::Writer> ImplPtr;

        protected:
            ImplPtr impl_;


        public:
            explicit Writer(const std::string &file_name);
            explicit Writer(std::ostream &output_stream);


            const serialization::Features &getSerializationFeatures() const;


            void flush();


            void descend(const std::string &map_name);
            void ascend();


            void startArray(const std::size_t size, const bool compact = false);
            void shiftArray();
            void endArray();


            void startMatrix(const bool compact = false);
            void startMatrixRow();
            void endMatrixRow();
            void endMatrix();


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_octave
}  // namespace ariles2


namespace ariles2
{
    /**
     * @brief Octave visitor.
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE octave
    {
        typedef ariles2::cfgwrite::Visitor<ns_octave::Writer> Writer;
    };
}  // namespace ariles2
