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

#include <iostream>

namespace ariles2
{
    namespace ns_msgpack_compact
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader;
        }


        /**
         * @brief Configuration reader class
         */
        class ARILES2_VISIBILITY_ATTRIBUTE Reader : public ns_msgpack::Base<ariles2::read::Visitor, impl::Reader>
        {
        public:
            /**
             * @brief Constructor
             *
             * @param[in] file_name
             */
            explicit Reader(const std::string &file_name);


            /**
             * @brief Constructor
             *
             * @param[in] input_stream
             */
            explicit Reader(std::istream &input_stream);


            void startMap(
                    const SizeLimitEnforcementType limit_type = SIZE_LIMIT_NONE,
                    const std::size_t min = 0,
                    const std::size_t max = 0);
            void endMap();

            void ascend();
            bool descend(const std::string &);


            std::size_t startArray();
            void endArray();
            void startArrayElement();
            void endArrayElement();


#define ARILES2_BASIC_TYPE(type) void readElement(type &element);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_msgpack_compact
}  // namespace ariles2
