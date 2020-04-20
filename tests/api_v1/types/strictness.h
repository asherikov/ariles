/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableStrictness1 : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "unique_id_on_a_particular_level_in_a_configuration_file"
        #define ARILES_CONSTRUCTOR ConfigurableStrictness1
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(real,        double)
        #include ARILES_INITIALIZE


        public:
            ConfigurableStrictness1()
            {
                setDefaults();
            }


            /**
             * @brief This method must be defined
             */
            virtual void setDefaults()
            {
                real_ = 1.33;
            }
    };


    class ConfigurableStrictness2 : public ConfigurableStrictness1
    {
        #define ARILES_SECTION_ID "unique_id_on_a_particular_level_in_a_configuration_file"
        #define ARILES_CONSTRUCTOR ConfigurableStrictness2
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_PARENT(ConfigurableStrictness1)
        #include ARILES_INITIALIZE


        public:
            ConfigurableStrictness2()
            {
                setDefaults();
            }


            /**
             * @brief This method must be defined
             */
            virtual void setDefaults()
            {
                integer_ = 10;
                ConfigurableStrictness1::setDefaults();
            }
    };
}
