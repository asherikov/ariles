/**
    @file
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles2
{
    namespace ns_msgpack
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer;
        }


        /**
         * @brief Configuration writer class
         */
        class ARILES2_VISIBILITY_ATTRIBUTE Writer : public ns_msgpack::Base<ariles2::write::Visitor, impl::Writer>
        {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] file_name
             */
            explicit Writer(const std::string &file_name);


            /**
             * @brief Constructor
             *
             * @param[out] output_stream
             */
            explicit Writer(std::ostream &output_stream);


            void descend(const std::string &map_name);


            void startMap(const std::string & /*id*/, const std::size_t num_entries);


            void flush();


            void startArray(const std::size_t size, const bool /*compact*/ = false);


            void startRoot(const std::string &name);
            void endRoot(const std::string &name);


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_msgpack
}  // namespace ariles2
