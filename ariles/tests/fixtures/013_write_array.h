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

                    BOOST_CHECK_EQUAL(writer.index_, writer.name_value_pairs_->size());

                    for (std::size_t i = 0; i < writer.name_value_pairs_->size(); ++i)
                    {
                        std::cout << (*writer.name_value_pairs_)[i].first << " = " << (*writer.name_value_pairs_)[i].second << std::endl;
                    }
                }


                // External buffers
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    std::vector<ariles::bridge::array::NameValuePair> name_value_pairs;

                    typename t_Bridge::Writer writer(&name_value_pairs, getWriterInitializer("configurable.cfg"));
                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, name_value_pairs.size());
                }


                // External buffers + reset
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    std::vector<ariles::bridge::array::NameValuePair> name_value_pairs;

                    typename t_Bridge::Writer writer(&name_value_pairs, getWriterInitializer("configurable.cfg"));
                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, name_value_pairs.size());

                    std::vector<ariles::bridge::array::NameValuePair> name_value_pairs_back = name_value_pairs;
                    writer.reset();

                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, name_value_pairs.size());

                    BOOST_CHECK_EQUAL(name_value_pairs_back.size(), name_value_pairs.size());

                    for (std::size_t i = 0; i < name_value_pairs.size(); ++i)
                    {
                        BOOST_CHECK_EQUAL(name_value_pairs_back[i].first, name_value_pairs[i].first);
                        BOOST_CHECK_EQUAL(name_value_pairs_back[i].second, name_value_pairs[i].second);
                    }
                }

                // External buffers + reset (initialize_structure = false)
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    const bool initialize_structure = false;
                    std::vector<ariles::bridge::array::NameValuePair> name_value_pairs;

                    typename t_Bridge::Writer writer(&name_value_pairs, getWriterInitializer("configurable.cfg"));
                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, name_value_pairs.size());

                    // ---

                    std::vector<ariles::bridge::array::NameValuePair> name_value_pairs_back = name_value_pairs;
                    writer.reset(initialize_structure);

                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, name_value_pairs.size());
                    BOOST_CHECK_EQUAL(name_value_pairs_back.size(), name_value_pairs.size());

                    for (std::size_t i = 0; i < name_value_pairs.size(); ++i)
                    {
                        BOOST_CHECK_EQUAL(name_value_pairs_back[i].first, name_value_pairs[i].first);
                        BOOST_CHECK_EQUAL(name_value_pairs_back[i].second, name_value_pairs[i].second);
                    }

                    // ---

                    writer.reset(initialize_structure);
                    name_value_pairs.clear();

                    configurable.writeConfig(writer);

                    BOOST_CHECK_EQUAL(writer.index_, name_value_pairs.size());
                    BOOST_CHECK_EQUAL(name_value_pairs_back.size(), name_value_pairs.size());

                    for (std::size_t i = 0; i < name_value_pairs.size(); ++i)
                    {
                        BOOST_CHECK_EQUAL(name_value_pairs[i].first, "");
                        BOOST_CHECK_EQUAL(name_value_pairs_back[i].second, name_value_pairs[i].second);
                    }
                }
            }
    };
}
