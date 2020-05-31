/**
    @file
    @author Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <string>
#include <vector>


namespace ariles2
{
    namespace ns_graphviz
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

            void startRoot(const std::string &name);
            void endRoot(const std::string &name);

            void descend(const std::string &map_name);
            void ascend();


            void startMap(const std::size_t num_entries);
            void endMap();

            void startArray(const std::size_t size, const bool compact = false);
            void shiftArray();
            void endArray();


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_graphviz
}  // namespace ariles2
