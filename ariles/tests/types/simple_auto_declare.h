/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


/**
 * @brief Short definition of a configurable class -- types of members are
 * passed to Ariles for automatic declaration.
 */
class ConfigurableAutoDeclare : public ariles::ConfigurableBase
{
    // conditionally optional, see ConfigurableNoAutoID
    // it is recommended to set it to the class name
    #define ARILES_SECTION_ID "unique_id_on_a_particular_level_in_a_configuration_file"
    // optional, create constructors to initialize using readConfig()
    #define ARILES_CONSTRUCTOR ConfigurableAutoDeclare
    // optional, override default flags
    #define ARILES_CONFIGURABLE_FLAGS   ariles::ConfigurableFlags::DEFAULT \
                                        | ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED \
                                        | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED
    // optional, but what is the point in omitting it?
    // members can be defined manually, see ConfigurableVerbose
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer,     int) \
        ARILES_TYPED_ENTRY_(real,        double)
    // mandatory
    #include ARILES_INITIALIZE


    public:
        ConfigurableAutoDeclare()
        {
            setDefaults();
        }


        /**
         * @brief This method must be defined
         */
        virtual void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
        }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
            finalize();
        }
#endif
};
