/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    template<class t_Configurable>
        class ConfigurableVector : public ariles::Base
    {
        #define ARILES_DEFAULT_ID "ConfigurableVector"
        #define ARILES_ENTRIES \
            ARILES_ENTRY_(vector)
        #include ARILES_INITIALIZE

        public:
            std::vector<t_Configurable> vector_;


        public:
            ConfigurableVector()
            {
                ariles::apply<ariles::defaults::Visitor>(*this);
            }


            void arilesVisit(   const ariles::defaults::Visitor &/*visitor*/,
                                const ariles::defaults::Visitor::Parameters &/*param*/)
            {
            }


            void randomize()
            {
                vector_.resize(4);

                for(std::size_t i = 0; i < vector_.size(); ++i)
                {
                    vector_[i].randomize();
                }
            }
    };


    template<class t_FixtureBase>
    class ComparisonVectorFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getWriterInitializer;
            using t_FixtureBase::getReaderInitializer;


        protected:
            template<class t_Configurable, class t_Bridge>
                void test()
            {
                ConfigurableVector<t_Configurable> configurable_vector_out;
                configurable_vector_out.randomize();
                BOOST_CHECK_NO_THROW(
                    ariles::apply<typename t_Bridge::Writer>(getWriterInitializer("configurable_match_vector.cfg"), configurable_vector_out);
                );

                // -------

                ConfigurableVector<t_Configurable> configurable_vector_in;
                BOOST_CHECK_NO_THROW(
                    ariles::apply<typename t_Bridge::Reader>(getReaderInitializer("configurable_match_vector.cfg"), configurable_vector_in);
                );

                // -------

                BOOST_REQUIRE_EQUAL(configurable_vector_out.vector_.size(), configurable_vector_in.vector_.size());
                for(std::size_t i = 0; i < configurable_vector_out.vector_.size(); ++i)
                {
                    compare(configurable_vector_out.vector_[i], configurable_vector_in.vector_[i]);
                }

                ariles::compare::Visitor visitor;
                ariles::compare::Visitor::Parameters param;
                param.double_tolerance_ = g_tolerance;
                param.compare_number_of_entries_ = true;
                param.throw_on_error_ = true;
                BOOST_CHECK(visitor.compare(configurable_vector_out, configurable_vector_in, param));
            }
    };
}