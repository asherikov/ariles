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

namespace ariles
{
    namespace bridge
    {
        namespace msgpack
        {
            namespace impl
            {
                class ARILES_LIB_LOCAL Reader;
            }


            /**
             * @brief Configuration reader class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Reader :
                public msgpack::Base<ariles::read::Visitor, impl::Reader>
            {
                protected:
                    std::size_t getMapSize(const bool /*expect_empty*/);


                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] file_name
                     */
                    explicit Reader(const std::string& file_name);


                    /**
                     * @brief Constructor
                     *
                     * @param[in] input_stream
                     */
                    explicit Reader(std::istream & input_stream);


                    bool descend(const std::string & child_name);
                    void ascend();


                    std::size_t startArray();
                    void endArray();
                    void shiftArray();


                    #define ARILES_BASIC_TYPE(type) \
                        void readElement(type &element);

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
