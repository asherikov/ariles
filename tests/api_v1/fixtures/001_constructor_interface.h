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
    template <class t_FixtureBase>
    class ConstructorInterfaceFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            BOOST_CHECK_NO_THROW(
                    t_Configurable configurable; configurable.randomize();

                    typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                    configurable.writeConfig(writer););

            BOOST_CHECK_NO_THROW(
                    typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                    t_Configurable configurable(reader););
        }
    };
}  // namespace ariles_tests
