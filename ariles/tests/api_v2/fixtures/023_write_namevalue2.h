/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <iostream>
#include <vector>
#include <utility>


namespace ariles_tests
{
    template <class t_FixtureBase>
    class NameValue2Fixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getWriterInitializer;

        class NameValueVector : public ariles2::namevalue2::NameValueContainer
        {
        public:
            std::vector<std::pair<std::string, double>> name_value_pairs_;

        public:
            std::string &name(const std::size_t index) override
            {
                return (name_value_pairs_[index].first);
            }
            double &value(const std::size_t index) override
            {
                return (name_value_pairs_[index].second);
            }

            void reserve(const std::size_t size) override
            {
                name_value_pairs_.reserve(size);
            }
            std::size_t size() const override
            {
                return (name_value_pairs_.size());
            }
            void resize(const std::size_t size) override
            {
                name_value_pairs_.resize(size);
            }
        };



    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            // Explicit instantiation of reader and writer classes
            {
                t_Configurable configurable;
                configurable.randomize();

                std::shared_ptr<NameValueVector> data = std::make_shared<NameValueVector>();
                typename t_Visitor::Writer writer(data);
                ariles2::apply(writer, configurable);

                for (std::size_t i = 0; i < data->size(); ++i)
                {
                    std::cout << data->name(i) << " = " << data->value(i) << std::endl;
                }
            }


            // Explicit instantiation of reader and writer classes + id
            {
                t_Configurable configurable;
                configurable.randomize();

                std::shared_ptr<NameValueVector> data = std::make_shared<NameValueVector>();
                typename t_Visitor::Writer writer(data);
                ariles2::apply(writer, configurable, std::string("id"));

                for (std::size_t i = 0; i < data->size(); ++i)
                {
                    std::cout << data->name(i) << " = " << data->value(i) << std::endl;
                }
            }


            // External buffers + reset
            {
                t_Configurable configurable;
                configurable.randomize();

                std::shared_ptr<NameValueVector> data = std::make_shared<NameValueVector>();
                typename t_Visitor::Writer writer(data);
                ariles2::apply(writer, configurable);


                NameValueVector name_value_pairs_back = *data;

                ariles2::apply(writer, configurable);


                BOOST_CHECK_EQUAL(name_value_pairs_back.size(), data->size());

                for (std::size_t i = 0; i < name_value_pairs_back.size(); ++i)
                {
                    BOOST_CHECK_EQUAL(name_value_pairs_back.name(i), data->name(i));
                    BOOST_CHECK_EQUAL(name_value_pairs_back.value(i), data->value(i));
                }
            }

            // External buffers + reset (initialize_names = false)
            {
                t_Configurable configurable;
                configurable.randomize();

                ariles2::write::Parameters param;
                param.persistent_structure_ = true;
                std::shared_ptr<NameValueVector> data = std::make_shared<NameValueVector>();
                typename t_Visitor::Writer writer(data);
                ariles2::apply(writer, configurable, param);


                // ---

                NameValueVector name_value_pairs_back = *data;

                ariles2::apply(writer, configurable, param);

                BOOST_CHECK_EQUAL(name_value_pairs_back.size(), data->size());

                for (std::size_t i = 0; i < data->size(); ++i)
                {
                    BOOST_CHECK_EQUAL(name_value_pairs_back.name(i), data->name(i));
                    BOOST_CHECK_EQUAL(name_value_pairs_back.value(i), data->value(i));
                }

                // ---

                data->resize(0);

                ariles2::apply(writer, configurable, param);

                BOOST_CHECK_EQUAL(name_value_pairs_back.size(), data->size());

                for (std::size_t i = 0; i < data->size(); ++i)
                {
                    BOOST_CHECK_EQUAL(name_value_pairs_back.name(i), data->name(i));
                    BOOST_CHECK_EQUAL(name_value_pairs_back.value(i), data->value(i));
                }

                // ---

                for (std::size_t i = 0; i < data->size(); ++i)
                {
                    data->name(i) = "";
                }

                ariles2::apply(writer, configurable, param);

                BOOST_CHECK_EQUAL(name_value_pairs_back.size(), data->size());

                for (std::size_t i = 0; i < data->size(); ++i)
                {
                    BOOST_CHECK_EQUAL(data->name(i), "");
                    BOOST_CHECK_EQUAL(name_value_pairs_back.value(i), data->value(i));
                }
            }
        }
    };
}  // namespace ariles_tests
