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
    /**
     * @brief Configurable class without extra constructors.
     */
    class ConfigurableNoConstructors : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "unique_id_on_a_particular_level_in_a_configuration_file"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(integer,     int) \
            ARILES_TYPED_ENTRY_(real,        double)
        #include ARILES_INITIALIZE


        public:
            ConfigurableNoConstructors()
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
}
