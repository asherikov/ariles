/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles2
{
    namespace ns_pugixml
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader;
        }


        /**
         * @brief Configuration reader class
         */
        class ARILES2_VISIBILITY_ATTRIBUTE Reader : public ns_pugixml::Base<ariles2::read::Visitor, impl::Reader>
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


            bool descend(const std::string &child_name);
            void ascend();


            bool getMapEntryNames(std::vector<std::string> &child_names);


            std::size_t startArray();
            void startArrayElement();
            void endArrayElement();
            void endArray();

            bool startRoot(const std::string &name);
            void endRoot(const std::string &name);


#define ARILES2_BASIC_TYPE(type) void readElement(type &element);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_pugixml
}  // namespace ariles2
