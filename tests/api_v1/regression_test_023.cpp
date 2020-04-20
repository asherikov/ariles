/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "../utility.h"


#include "ariles/bridges/rapidjson.h"

// If no format header is included, ariles is disabled, and
// ariles::ConfigurableBase is just a dummy class.
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

#include "types/simple_floats.h"


// ===============================================================
// FIXTURES
// ===============================================================

#undef ARILES_BRIDGE_ros
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
                    string_id_ = "regression_test_023_string.json";
                }
        };

        class FilenameReaderBaseFloat
        {
            public:
                std::string string_id_;

            public:
                FilenameReaderBaseFloat()
                {
                    string_id_ = "regression_test_023_float.json";
                }
        };

        typedef FilenameReaderInitializer<FilenameReaderBaseString> FilenameReaderInitializer023_String;
        typedef FilenameReaderInitializer<FilenameReaderBaseFloat> FilenameReaderInitializer023_Float;
    }


    template<class t_FixtureBase>
    class WriteDiffFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getReaderInitializer;


        protected:
            template<class t_Configurable, class t_Visitor>
                void test()
            {
                t_Configurable configurable;

                BOOST_CHECK_NO_THROW(
                    std::ofstream output_file_stream;
                    output_file_stream.open("configurable.cfg");
                    typename t_Visitor::Writer visitor(
                        output_file_stream,
                        ariles::rapidjson::Flags::DISABLE_STRING_FLOATS);
                );

                BOOST_CHECK_NO_THROW(
                    std::ifstream input_file_stream;
                    input_file_stream.open("regression_test_023_float.json");
                    typename t_Visitor::Reader visitor(
                        input_file_stream,
                        ariles::rapidjson::Flags::DISABLE_STRING_FLOATS);
                );

                BOOST_CHECK_NO_THROW(
                    typename t_Visitor::Writer visitor(
                        "configurable.cfg",
                        ariles::rapidjson::Flags::DISABLE_STRING_FLOATS);
                    configurable.writeConfig(visitor);
                );

                BOOST_CHECK_EQUAL(0, system("cmp configurable.cfg regression_test_023_float.json"));


                BOOST_CHECK_NO_THROW(
                    typename t_Visitor::Writer visitor("configurable.cfg");
                    configurable.writeConfig(visitor);
                );

                BOOST_CHECK_EQUAL(0, system("cmp configurable.cfg regression_test_023_string.json"));
            }
    };
}

// ===============================================================
// TESTS
// ===============================================================


ARILES_FIXTURE_TEST_CASE(ReadCompareFixture, rapidjson, rapidjson, ConfigurableSimpleFloats, FilenameReaderInitializer023_String)
ARILES_FIXTURE_TEST_CASE(ReadCompareFixture, rapidjson, rapidjson, ConfigurableSimpleFloats, FilenameReaderInitializer023_Float)
ARILES_FIXTURE_TEST_CASE(WriteDiffFixture, rapidjson, rapidjson, ConfigurableSimpleFloats, FilenameInitializer)
