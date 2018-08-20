/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once


class Configurable : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "Configurable"
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer,     int) \
        ARILES_TYPED_ENTRY_(real,        double)
    #include ARILES_INITIALIZE

    public:
        double another_real_;

    public:
        Configurable()
        {
            setDefaults();
            finalize();
        }

        virtual ~Configurable() {}


        virtual void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
        }


        virtual void finalize()
        {
            another_real_ = integer_ * real_;
        }


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_    = GET_RANDOM_REAL;
            finalize();
        }
};
