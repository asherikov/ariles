/**
    @file
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <msgpack.hpp>
#include <ariles2/visitors/msgpack.h>


namespace ariles2
{
    namespace ns_msgpack
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer
            {
            private:
                Writer(const Writer &);
                void operator=(const Writer &);


            public:
                /// output file stream
                std::ofstream config_ofs_;

                /// output stream
                std::ostream *output_stream_;

                ::msgpack::packer<std::ostream> *packer_;

                std::size_t nameless_counter_;


            public:
                explicit Writer(const std::string &file_name)
                {
                    ariles2::write::Visitor::openFile(config_ofs_, file_name);
                    output_stream_ = &config_ofs_;
                    packer_ = new ::msgpack::packer<std::ostream>(*output_stream_);

                    nameless_counter_ = 0;
                }


                explicit Writer(std::ostream &output_stream)
                {
                    output_stream_ = &output_stream;
                    packer_ = new ::msgpack::packer<std::ostream>(*output_stream_);

                    nameless_counter_ = 0;
                }


                ~Writer()
                {
                    delete packer_;
                }
            };
        }  // namespace impl
    }      // namespace ns_msgpack
}  // namespace ariles2


namespace ariles2
{
    namespace ns_msgpack
    {
        Writer::Writer(const std::string &file_name)
        {
            impl_ = ImplPtr(new Impl(file_name));
        }


        Writer::Writer(std::ostream &output_stream)
        {
            impl_ = ImplPtr(new Impl(output_stream));
        }


        void Writer::startMap(const Parameters &, const std::size_t num_entries)
        {
            ARILES2_TRACE_FUNCTION;
            impl_->packer_->pack_map(num_entries);
        }

        void Writer::startMapEntry(const std::string &map_name)
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_TRACE_VALUE(map_name);
            impl_->packer_->pack(map_name);
        }


        void Writer::flush()
        {
            ARILES2_TRACE_FUNCTION;
            impl_->output_stream_->flush();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(size <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");

            impl_->packer_->pack_array(size);
        }


        void Writer::startRoot(const std::string &name, const Parameters &)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == name.empty())
            {
                ARILES2_ASSERT(
                        0 == impl_->nameless_counter_,
                        "Multiple nameless root entries are not supported, specify root names explicitly.");
                ++impl_->nameless_counter_;
                impl_->packer_->pack_map(1);
                startMapEntry("ariles");
            }
            else
            {
                impl_->packer_->pack_map(1);
                startMapEntry(name);
            }
        }

        void Writer::endRoot(const std::string & /*name*/)
        {
            ARILES2_TRACE_FUNCTION;
            endMapEntry();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        impl_->packer_->pack(element);                                                                                 \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_msgpack
}  // namespace ariles2
