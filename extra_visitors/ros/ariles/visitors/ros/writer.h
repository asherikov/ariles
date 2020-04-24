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
    namespace ns_ros
    {
        namespace impl
        {
            class ARILES_LIB_LOCAL Writer;
        }



        /**
         * @brief Configuration writer class
         */
        class ARILES_LIB_EXPORT Writer : public ns_ros::Base<ariles::write::Visitor, impl::Writer>
        {
        public:
            explicit Writer(const ::ros::NodeHandle &nh);


            void flush();



            void descend(const std::string &map_name);
            void ascend();

            void startArray(const std::size_t size, const bool /*compact*/ = false);
            void shiftArray();
            void endArray();


            void startRoot(const std::string &name);
            void endRoot(const std::string &name);


#define ARILES_BASIC_TYPE(type) void writeElement(const type &element);

            ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

#undef ARILES_BASIC_TYPE
        };
    }  // namespace ns_ros
}  // namespace ariles
