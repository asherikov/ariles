/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


/**
 * @brief Configurable class without extra constructors.
 */
class ConfigurableNoAutoID : public ariles::ConfigurableBase
{
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer,     int) \
        ARILES_TYPED_ENTRY_(real,        double)
    #include ARILES_INITIALIZE

    protected:
        std::string id_;

    public:
        ConfigurableNoAutoID()
        {
            setDefaults();
            id_ = "unique_id_on_a_particular_level_in_a_configuration_file";
        }


        /**
         * @brief This method must be defined
         */
        virtual void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
        }


        /**
         * @brief This method must be implmented if ARILES_SECTION_ID is not defined.
         *
         * @return id
         */
        const std::string & getConfigSectionID() const
        {
            return (id_);
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
