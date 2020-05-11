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
    namespace ns_msgpack_compact
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

            public:
                explicit Writer(const std::string &file_name)
                {
                    ariles2::write::Visitor::openFile(config_ofs_, file_name);
                    output_stream_ = &config_ofs_;
                    packer_ = new ::msgpack::packer<std::ostream>(*output_stream_);
                }


                explicit Writer(std::ostream &output_stream)
                {
                    output_stream_ = &output_stream;
                    packer_ = new ::msgpack::packer<std::ostream>(*output_stream_);
                }


                ~Writer()
                {
                    delete packer_;
                }
            };
        }  // namespace impl
    }      // namespace ns_msgpack_compact
}  // namespace ariles2


namespace ariles2
{
    namespace ns_msgpack_compact
    {
        Writer::Writer(const std::string &file_name)
        {
            impl_ = ImplPtr(new Impl(file_name));
        }


        Writer::Writer(std::ostream &output_stream)
        {
            impl_ = ImplPtr(new Impl(output_stream));
        }


        void Writer::startMap(const std::size_t num_entries)
        {
            impl_->packer_->pack_array(num_entries);
        }


        void Writer::flush()
        {
            impl_->output_stream_->flush();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            ARILES2_ASSERT(size <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");
            impl_->packer_->pack_array(size);
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element)                                                                     \
    {                                                                                                                  \
        impl_->packer_->pack(element);                                                                                 \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_msgpack_compact
}  // namespace ariles2
