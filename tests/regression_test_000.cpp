/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

// Enable YAML configuration files (must be first)
#include "ariles/format_yaml.h"
#include "ariles/format_msgpack.h"
// common & abstract classes
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
    #include ARILES_DEFINE_ACCESSORS


    public:
        ConfigurableAutoDeclare()
        {
            setDefaults();
        }


        /**
         * @brief This method must be defined
         */
        void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
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
    #include ARILES_DEFINE_ACCESSORS


    public:
        int                     integer_;
        double                  real_;


    public:
        ConfigurableVerbose()
        {
            setDefaults();
        }


        void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
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
    #include ARILES_DEFINE_ACCESSORS


    public:
        ConfigurableNoConstructors()
        {
            setDefaults();
        }


        /**
         * @brief This method must be defined
         */
        void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
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
    #include ARILES_DEFINE_ACCESSORS

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
        void setDefaults()
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
};


/**
 * @brief Configurable class without extra constructors.
 */
class ConfigurableEmpty : public ariles::ConfigurableBase
{
    #define ARILES_ENTRIES
    #include ARILES_DEFINE_ACCESSORS

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
        void setDefaults()
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
};


// ===============================================================
// FIXTURES
// ===============================================================

#include "fixture_000_basic_interface.h"
#include "fixture_001_constructor_interface.h"



// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(NAMESPACE) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableVerbose) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableAutoDeclare) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableNoConstructors) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableNoAutoID) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, ConfigurableEmpty) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, NAMESPACE, ConfigurableVerbose) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, NAMESPACE, ConfigurableAutoDeclare)

ARILES_TESTS(msgpack)
ARILES_TESTS(yaml)
