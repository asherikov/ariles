/**
    @file
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <limits>

#include <ariles2/visitors/msgpack.h>
#include "writer_common.h"


namespace ariles2
{
    namespace ns_msgpack_compact
    {
        using ns_msgpack_c::packValue;


        namespace impl
        {
            class Writer : public write::FileVisitorImplementation
            {
            public:
                msgpack_packer packer_;


            public:
                Writer(const Writer &) = delete;
                void operator=(const Writer &) = delete;


                template <class... t_Args>
                explicit Writer(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                    msgpack_packer_init(&packer_, output_stream_, ns_msgpack_c::writeToStream);
                }
            };
        }  // namespace impl
    }  // namespace ns_msgpack_compact
}  // namespace ariles2


namespace ariles2
{
    namespace ns_msgpack_compact
    {
        Writer::Writer(const std::string &file_name)
        {
            makeImplPtr(file_name);
        }


        Writer::Writer(std::ostream &output_stream)
        {
            makeImplPtr(output_stream);
        }


        void Writer::startMap(const Parameters &, const std::size_t num_entries)
        {
            msgpack_pack_array(&impl_->packer_, num_entries);
        }


        void Writer::flush()
        {
            impl_->output_stream_->flush();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            CPPUT_ASSERT(size <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");
            msgpack_pack_array(&impl_->packer_, size);
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        packValue(&impl_->packer_, element);                                                                           \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_msgpack_compact
}  // namespace ariles2
