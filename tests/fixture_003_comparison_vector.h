/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


template<class t_Configurable>
    class ConfigurableVector : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableVector"
    #define ARILES_ENTRIES \
        ARILES_ENTRY_(vector)
    #include ARILES_DEFINE_ACCESSORS

    public:
        std::vector<t_Configurable> vector_;


    public:
        ConfigurableVector()
        {
            setDefaults();
        }


        void setDefaults()
        {
            vector_.resize(4);
        }
};


class ComparisonVectorFixture
{
    protected:
        template<class t_Configurable, class t_Reader, class t_Writer>
            void test()
        {
            ConfigurableVector<t_Configurable> configurable_vector_out;
            BOOST_CHECK_NO_THROW(
                configurable_vector_out.template writeConfig<t_Writer>("configurable_match_vector.cfg");
            );

            // -------

            ConfigurableVector<t_Configurable> configurable_vector_in;
            BOOST_CHECK_NO_THROW(
                configurable_vector_in.template readConfig<t_Reader>("configurable_match_vector.cfg");
            );

            // -------

            BOOST_CHECK_EQUAL(configurable_vector_out.vector_.size(), configurable_vector_in.vector_.size());
            for(std::size_t i = 0; i < configurable_vector_out.vector_.size(); ++i)
            {
                compare(configurable_vector_out.vector_[i], configurable_vector_in.vector_[i]);
            }
        }
};
