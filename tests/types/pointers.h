/**
    @file
    @author  Alexander Sherikov
    @copyright

    @brief
*/

#pragma once


class ConfigurablePointers : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurablePointers"
    #define ARILES_CONSTRUCTOR ConfigurablePointers


    #define ARILES_ENTRIES_0

#if __cplusplus >= 201103L
    #define ARILES_ENTRIES_1 \
        ARILES_ENTRIES_0 \
        ARILES_TYPED_ENTRY_(std_shared_ptr_real,          std::shared_ptr<double>) \
        ARILES_TYPED_ENTRY_(std_unique_ptr_real,          std::unique_ptr<double>)
#else
    #define ARILES_ENTRIES_1 ARILES_ENTRIES_0
#endif


#ifdef ARILES_ADAPTER_BOOSTPTR
    #define ARILES_ENTRIES_2 \
        ARILES_ENTRIES_1 \
        ARILES_TYPED_ENTRY_(shared_ptr_real,          boost::shared_ptr<double>) \
        ARILES_TYPED_ENTRY_(shared_ptr_real_null,     boost::shared_ptr<double>) \
        ARILES_TYPED_ENTRY_(unique_ptr_real,          boost::movelib::unique_ptr<double>)
#else
    #define ARILES_ENTRIES_2 ARILES_ENTRIES_1
#endif


    #define ARILES_ENTRIES ARILES_ENTRIES_2
    #include ARILES_INITIALIZE

#undef ARILES_ENTRIES_0
#undef ARILES_ENTRIES_1
#undef ARILES_ENTRIES_2


    public:
        ConfigurablePointers()
        {
            setDefaults();
        }

        virtual void setDefaults()
        {

#if __cplusplus >= 201103L
            std_shared_ptr_real_.reset();
            std_unique_ptr_real_.reset();
#endif

#ifdef ARILES_ADAPTER_BOOSTPTR
            shared_ptr_real_.reset();
            shared_ptr_real_null_.reset();
            unique_ptr_real_.reset();
#endif
        }


        void randomize()
        {
#if __cplusplus >= 201103L
            std_shared_ptr_real_ = std::make_shared<double>();
            *std_shared_ptr_real_ = GET_RANDOM_REAL;

            std_unique_ptr_real_.reset(new double());
            *std_unique_ptr_real_ = GET_RANDOM_REAL;
#endif

#ifdef ARILES_ADAPTER_BOOSTPTR
            shared_ptr_real_ = boost::make_shared<double>();
            unique_ptr_real_ = boost::movelib::make_unique<double>();

            *shared_ptr_real_ = GET_RANDOM_REAL;

            shared_ptr_real_null_.reset();
#endif
        }
};


template<class t_Configurable_out, class t_Configurable_in>
void    compare(const t_Configurable_out    &configurable_out,
                const t_Configurable_in     &configurable_in)
{
#if __cplusplus >= 201103L
    if (configurable_in.std_shared_ptr_real_ == NULL)
    {
        BOOST_CHECK_EQUAL(configurable_out.std_shared_ptr_real_, configurable_in.std_shared_ptr_real_);
    }
    else
    {
        BOOST_CHECK(configurable_out.std_shared_ptr_real_ != NULL);
        BOOST_CHECK_CLOSE(*configurable_out.std_shared_ptr_real_, *configurable_in.std_shared_ptr_real_, g_tolerance);
    }
    if (configurable_in.std_unique_ptr_real_ == NULL)
    {
        BOOST_CHECK(configurable_out.std_unique_ptr_real_ == configurable_in.std_unique_ptr_real_);
    }
    else
    {
        BOOST_CHECK(configurable_out.std_unique_ptr_real_ != NULL);
        BOOST_CHECK_CLOSE(*configurable_out.std_unique_ptr_real_, *configurable_in.std_unique_ptr_real_, g_tolerance);
    }
#endif


#ifdef ARILES_ADAPTER_BOOSTPTR
    if (configurable_in.shared_ptr_real_ == NULL)
    {
        BOOST_CHECK_EQUAL(configurable_out.shared_ptr_real_, configurable_in.shared_ptr_real_);
    }
    else
    {
        BOOST_CHECK(configurable_out.shared_ptr_real_ != NULL);
        BOOST_CHECK_CLOSE(*configurable_out.shared_ptr_real_, *configurable_in.shared_ptr_real_, g_tolerance);
    }
    if (configurable_in.unique_ptr_real_ == NULL)
    {
        BOOST_CHECK(configurable_out.unique_ptr_real_ == configurable_in.unique_ptr_real_);
    }
    else
    {
        BOOST_CHECK(configurable_out.unique_ptr_real_ != NULL);
        BOOST_CHECK_CLOSE(*configurable_out.unique_ptr_real_, *configurable_in.unique_ptr_real_, g_tolerance);
    }

    BOOST_CHECK(configurable_out.shared_ptr_real_null_ == NULL);
    BOOST_CHECK_EQUAL(configurable_out.shared_ptr_real_null_, configurable_in.shared_ptr_real_null_);
#endif
}
