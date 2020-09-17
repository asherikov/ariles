/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

#include <ariles2/visitors/rapidjson.h>

// If no format header is included, ariles is disabled, and
// ariles2::ConfigurableBase is just a dummy class.
#include <ariles2/ariles.h>


// ===============================================================
// TYPES
// ===============================================================

#include "types/simple_floats.h"


// ===============================================================
// FIXTURES
// ===============================================================

#undef ARILES_VISITOR_rosparam
#include "fixtures/initializers.h"
#include "fixtures/015_read_compare.h"

namespace ariles_tests
{
    namespace initializers
    {
        class FilenameReaderBaseString
        {
        public:
            std::string string_id_;

        public:
            FilenameReaderBaseString()
            {
                string_id_ = "regression_test_223_string.json";
            }
        };

        class FilenameReaderBaseFloat
        {
        public:
            std::string string_id_;

        public:
            FilenameReaderBaseFloat()
            {
                string_id_ = "regression_test_223_float.json";
            }
        };

        typedef FilenameReaderInitializer<FilenameReaderBaseString> FilenameReaderInitializer223_String;
        typedef FilenameReaderInitializer<FilenameReaderBaseFloat> FilenameReaderInitializer223_Float;
    }  // namespace initializers


    template <class t_FixtureBase>
    class WriteDiffFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            t_Configurable configurable;


            BOOST_CHECK_NO_THROW({
                typename t_Visitor::Writer::Parameters parameters;
                parameters.write_.fallback_to_string_floats_ = false;
                typename t_Visitor::Writer writer("configurable.cfg");
                ariles2::apply(writer, configurable, parameters);
            });

            BOOST_CHECK_EQUAL(0, system("cmp configurable.cfg regression_test_223_float.json"));


            BOOST_CHECK_NO_THROW({
                typename t_Visitor::Writer writer("configurable.cfg");
                ariles2::apply(writer, configurable);
            });

            BOOST_CHECK_EQUAL(0, system("cmp configurable.cfg regression_test_223_string.json"));
        }
    };
}  // namespace ariles_tests

// ===============================================================
// TESTS
// ===============================================================


ARILES_FIXTURE_TEST_CASE(
        ReadCompareFixture,
        rapidjson,
        rapidjson,
        ConfigurableSimpleFloats,
        FilenameReaderInitializer223_String)
ARILES_FIXTURE_TEST_CASE(
        ReadCompareFixture,
        rapidjson,
        rapidjson,
        ConfigurableSimpleFloats,
        FilenameReaderInitializer223_Float)
ARILES_FIXTURE_TEST_CASE(WriteDiffFixture, rapidjson, rapidjson, ConfigurableSimpleFloats, FilenameInitializer)
