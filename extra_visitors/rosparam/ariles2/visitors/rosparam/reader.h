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
    namespace ns_rosparam
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Reader;
        }



        /**
         * @brief Configuration reader class
         */
        class ARILES2_VISIBILITY_ATTRIBUTE Reader : public serialization::PIMPLVisitor<read::Visitor, impl::Reader>
        {
        protected:
            bool startRoot(const std::string &name);
            void endRoot(const std::string &name);

        public:
            /**
             * @brief Constructor
             *
             * @param[in] nh NodeHandle
             */
            explicit Reader(const ::ros::NodeHandle &nh);


            void startMap(
                    const SizeLimitEnforcementType limit_type = SIZE_LIMIT_NONE,
                    const std::size_t min = 0,
                    const std::size_t max = 0);
            bool startMapEntry(const std::string &child_name);
            void endMapEntry();


            bool startIteratedMap(
                    const SizeLimitEnforcementType /*limit_type*/ = SIZE_LIMIT_NONE,
                    const std::size_t /*min*/ = 0,
                    const std::size_t /*max*/ = 0);
            bool startIteratedMapElement(std::string &entry_name);
            void endIteratedMapElement();
            void endIteratedMap();


            std::size_t startArray();
            void startArrayElement();
            void endArrayElement();
            void endArray();


#define ARILES2_BASIC_TYPE(type) void readElement(type &element);

            ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
        };
    }  // namespace ns_rosparam
}  // namespace ariles2
