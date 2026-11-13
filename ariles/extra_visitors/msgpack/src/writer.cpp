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
    namespace ns_msgpack
    {
        using ns_msgpack_c::packValue;


        namespace impl
        {
            class Writer : public write::FileVisitorImplementation
            {
            public:
                msgpack_packer packer_;

                std::size_t nameless_counter_;


            public:
                Writer(const Writer &) = delete;
                void operator=(const Writer &) = delete;


                template <class... t_Args>
                explicit Writer(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                    msgpack_packer_init(&packer_, output_stream_, ns_msgpack_c::writeToStream);

                    nameless_counter_ = 0;
                }
            };
        }  // namespace impl
    }  // namespace ns_msgpack
}  // namespace ariles2


namespace ariles2
{
    namespace ns_msgpack
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
            CPPUT_TRACE_FUNCTION;
            msgpack_pack_map(&impl_->packer_, num_entries);
        }

        void Writer::startMapEntry(const std::string &map_name)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_TRACE_VALUE(map_name);
            msgpack_pack_str(&impl_->packer_, map_name.size());
            msgpack_pack_str_body(&impl_->packer_, map_name.data(), map_name.size());
        }


        void Writer::flush()
        {
            CPPUT_TRACE_FUNCTION;
            impl_->output_stream_->flush();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            CPPUT_TRACE_FUNCTION;
            CPPUT_ASSERT(size <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");

            msgpack_pack_array(&impl_->packer_, size);
        }


        void Writer::startRoot(const std::string &name, const Parameters &)
        {
            CPPUT_TRACE_FUNCTION;
            if (name.empty())
            {
                CPPUT_ASSERT(
                        0 == impl_->nameless_counter_,
                        "Multiple nameless root entries are not supported, specify root names explicitly.");
                ++impl_->nameless_counter_;
                msgpack_pack_map(&impl_->packer_, 1);
                startMapEntry("ariles");
            }
            else
            {
                msgpack_pack_map(&impl_->packer_, 1);
                startMapEntry(name);
            }
        }

        void Writer::endRoot(const std::string & /*name*/)
        {
            CPPUT_TRACE_FUNCTION;
            endMapEntry();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        packValue(&impl_->packer_, element);                                                                           \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE
    }  // namespace ns_msgpack
}  // namespace ariles2
