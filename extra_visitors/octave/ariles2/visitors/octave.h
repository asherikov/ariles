/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup octave Octave
@ingroup config

@brief Serialize to Octave/MATLAB script.

@note Eigen matrices are written in the 'native' format, so they can be used
      directly, no reshaping is necessary.

@note Matlab might be supported, but has not been tested.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_octave

#include <ariles2/internal/helpers.h>
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


            void flush();


            void startMapEntry(const std::string &map_name);
            void endMapEntry();


            bool startIteratedMap(const std::size_t /*num_entries*/, const Parameters &)
            {
                return (false);
            }


            void startArray(const std::size_t size, const bool compact = false);
            void endArrayElement();
            void endArray();


            void startVector(const std::size_t size);
            void startVectorElement();
            void endVectorElement();
            void endVector();


            void startMatrix(
                    const bool dynamic,
                    const std::size_t cols,
                    const std::size_t rows,
                    const Parameters &param);
            void startMatrixRow();
            void startMatrixElement();
            void endMatrixElement();
            void endMatrixRow();
            void endMatrix(const bool dynamic);


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element, const Parameters &param);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)
            ARILES2_MACRO_SUBSTITUTE(ARILES2_COMPLEX_NUMBER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_octave
}  // namespace ariles2


namespace ariles2
{
    /**
     * @brief Octave visitor.
     * @ingroup octave
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE octave
    {
        typedef ariles2::cfgwrite::Visitor<ns_octave::Writer> Writer;
    };
}  // namespace ariles2
