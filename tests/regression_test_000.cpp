/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#ifdef ARILES_BRIDGE_yaml_cpp03
#include "ariles/bridges/yaml_cpp03.h"
#endif

#ifdef ARILES_BRIDGE_yaml_cpp
#include "ariles/bridges/yaml_cpp.h"
#endif

#ifdef ARILES_BRIDGE_msgpack
#include "ariles/bridges/msgpack.h"
#endif

#ifdef ARILES_BRIDGE_ros
#include "ariles/bridges/ros.h"
#endif

#ifdef ARILES_BRIDGE_rapidjson
#include "ariles/bridges/rapidjson.h"
#endif

#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================


/**
 * @brief Short definition of a configurable class -- types of members are
 * passed to Ariles for automatic declaration.
 */
class ConfigurableAutoDeclare : public ariles::ConfigurableBase
{
    // conditionally optional, see ConfigurableNoAutoID
    // it is recommended to set it to the class name
    #define ARILES_SECTION_ID "unique_id_on_a_particular_level_in_a_configuration_file"
    // optional
    #define ARILES_CONSTRUCTOR ConfigurableAutoDeclare
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


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
            finalize();
        }
};


/**
 * @brief Verbose definition of a configurable class (with explicit declaration
 * of members)
 */
class ConfigurableVerbose : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "unique_id_on_a_particular_level_in_a_configuration_file"
    #define ARILES_CONSTRUCTOR ConfigurableVerbose
    #define ARILES_ENTRIES \
        ARILES_ENTRY_(integer) \
        ARILES_ENTRY_(real)
    #include ARILES_INITIALIZE


    public:
        int                     integer_;
        double                  real_;


    public:
        ConfigurableVerbose()
        {
            setDefaults();
        }


        virtual void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
        }


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
            finalize();
        }
};


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


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
            finalize();
        }
};


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


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
            finalize();
        }
};


/**
 * @brief Configurable class without extra constructors.
 */
class ConfigurableEmpty : public ariles::ConfigurableBase
{
    #define ARILES_ENTRIES
    #include ARILES_INITIALIZE

    protected:
        std::string id_;

    public:
        ConfigurableEmpty()
        {
            setDefaults();
            id_ = "unique_id_on_a_particular_level_in_a_configuration_file";
        }


        /**
         * @brief This method must be defined
         */
        virtual void setDefaults()
        {
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


        void randomize()
        {
            finalize();
        }
};


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixtures/initializers.h"
#include "fixtures/000_basic_interface.h"
#include "fixtures/001_constructor_interface.h"



// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(BRIDGE_ID, NAMESPACE, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableAutoDeclare, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableNoConstructors, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableNoAutoID, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableEmpty, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableVerbose, INITIALIZER) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, BRIDGE_ID, NAMESPACE, ConfigurableAutoDeclare, INITIALIZER)


#include "instantiate.h"
