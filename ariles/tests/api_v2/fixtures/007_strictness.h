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
    class StrictnessFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Configurable1, class t_Configurable2, class t_Visitor>
        void test()
        {
            // Explicit instantiation of reader and writer classes
            BOOST_CHECK_NO_THROW(t_Configurable1 configurable;

                                 typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                                 ariles2::apply(writer, configurable););

            BOOST_CHECK_THROW(t_Configurable2 configurable;

                              typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                              ariles2::apply(reader, configurable);
                              , std::runtime_error);

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            BOOST_CHECK_NO_THROW(t_Configurable1 configurable; ariles2::apply<typename t_Visitor::Writer>(
                                         getWriterInitializer("configurable2.cfg"), configurable););

            BOOST_CHECK_THROW(t_Configurable2 configurable; ariles2::apply<typename t_Visitor::Reader>(
                                      getReaderInitializer("configurable2.cfg"), configurable);
                              , std::runtime_error);
        }
    };
}  // namespace ariles_tests
