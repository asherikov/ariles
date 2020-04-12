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
                class ARILES_LIB_LOCAL Writer;
            }


            /**
             * @brief Configuration writer class
             */
            class ARILES_LIB_EXPORT Writer :
                public msgpack::Base<ariles::write::Visitor, impl::Writer>
            {
                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] file_name
                     */
                    explicit Writer(const std::string& file_name);


                    /**
                     * @brief Constructor
                     *
                     * @param[out] output_stream
                     */
                    explicit Writer(std::ostream& output_stream);


                    void descend(const std::string &map_name);


                    void startMap(const std::size_t num_entries);


                    void initRoot();


                    void flush();


                    void startArray(const std::size_t size, const bool /*compact*/ = false);


                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type & element);

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}