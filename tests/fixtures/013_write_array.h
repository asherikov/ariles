/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <iostream>

namespace ariles_tests
{
    template<class t_FixtureBase>
    class ArrayFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getWriterInitializer;


        protected:
            template<class t_Configurable, class t_Bridge>
                void test()
            {
                // Exlicit instantiation of reader and writer classes
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    typename t_Bridge::Writer writer(getWriterInitializer("configurable.cfg"));
                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, writer.names_->size());
                    BOOST_CHECK_EQUAL(writer.index_, writer.values_->size());

                    for (std::size_t i = 0; i < writer.names_->size(); ++i)
                    {
                        std::cout << (*writer.names_)[i] << " = " << (*writer.values_)[i] << std::endl;
                    }
                }


                // External buffers + prefix
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    const std::string prefix = "blahprefix.";
                    std::vector<std::string> names;
                    std::vector<double> values;


                    typename t_Bridge::Writer writer(&names, &values, getWriterInitializer("configurable.cfg"), prefix);
                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, names.size());
                    BOOST_CHECK_EQUAL(writer.index_, values.size());

                    for (std::size_t i = 0; i < names.size(); ++i)
                    {
                        BOOST_CHECK_EQUAL(names[i].substr(0, prefix.size()), prefix);
                    }
                }


                // External buffers + reset
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    std::vector<std::string> names;
                    std::vector<double> values;

                    typename t_Bridge::Writer writer(&names, &values, getWriterInitializer("configurable.cfg"));
                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, names.size());
                    BOOST_CHECK_EQUAL(writer.index_, values.size());

                    std::vector<std::string> names_back = names;
                    std::vector<double> values_back = values;
                    writer.reset();

                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, names.size());
                    BOOST_CHECK_EQUAL(writer.index_, values.size());

                    BOOST_CHECK_EQUAL(names_back.size(), names.size());

                    for (std::size_t i = 0; i < names.size(); ++i)
                    {
                        BOOST_CHECK_EQUAL(names_back[i], names[i]);
                        BOOST_CHECK_EQUAL(values_back[i], values[i]);
                    }
                }

                // External buffers + reset (preserve_structure = true)
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    const bool preserve_structure = true;
                    std::vector<std::string> names;
                    std::vector<double> values;

                    typename t_Bridge::Writer writer(&names, &values, getWriterInitializer("configurable.cfg"));
                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, names.size());
                    BOOST_CHECK_EQUAL(writer.index_, values.size());

                    // ---

                    std::vector<std::string> names_back = names;
                    std::vector<double> values_back = values;
                    writer.reset(preserve_structure);

                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, names.size());
                    BOOST_CHECK_EQUAL(writer.index_, values.size());

                    BOOST_CHECK_EQUAL(names_back.size(), names.size());

                    for (std::size_t i = 0; i < names.size(); ++i)
                    {
                        BOOST_CHECK_EQUAL(names_back[i], names[i]);
                        BOOST_CHECK_EQUAL(values_back[i], values[i]);
                    }

                    // ---

                    writer.reset(preserve_structure);
                    names.clear();
                    values.clear();

                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, names.size());
                    BOOST_CHECK_EQUAL(writer.index_, values.size());

                    BOOST_CHECK_EQUAL(names_back.size(), names.size());

                    for (std::size_t i = 0; i < names.size(); ++i)
                    {
                        BOOST_CHECK_EQUAL(names[i], ""); // names are not reinitialized
                        BOOST_CHECK_EQUAL(values_back[i], values[i]);
                    }
                }
            }
    };
}
