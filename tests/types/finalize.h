/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once


class ConfigurableFinalize : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableFinalize"
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer,     int) \
        ARILES_TYPED_ENTRY_(real,        double)
    #include ARILES_INITIALIZE

    public:
        double another_real_;

    public:
        ConfigurableFinalize()
        {
            setDefaults();
            finalize();
        }

        virtual ~ConfigurableFinalize() {}


        virtual void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
        }


        virtual void finalize()
        {
            another_real_ = integer_ * real_;
        }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
            finalize();
        }
#endif
};
