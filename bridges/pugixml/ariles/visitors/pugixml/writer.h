/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    namespace bridge
    {
        namespace pugixml
        {
            namespace impl
            {
                class ARILES_LIB_LOCAL Writer;
            }


            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer :
                public pugixml::Base<ariles::write::Visitor, impl::Writer>
            {
                public:
                    explicit Writer(const std::string& file_name);
                    explicit Writer(std::ostream& output_stream);


                    void flush();


                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] map_name name of the map
                     */
                    void descend(const std::string &map_name);
                    void ascend();


                    void startArray(const std::size_t size, const bool /*compact*/ = false);
                    void shiftArray();
                    void endArray();



                    #define ARILES_BASIC_TYPE(type) \
                        void writeElement(const type & element);

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}
