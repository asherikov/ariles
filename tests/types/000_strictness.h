/**
    @file
    @author  Alexander Sherikov

    @copyright (c) 2018 PAL Robotics SL. All Rights Reserved

    @brief
*/

#pragma once


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
