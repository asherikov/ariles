#pragma once


#include <string>
#include <vector>


// NOLINTBEGIN(*)
namespace std_msgs
{
    template <class ContainerAllocator>
    struct Header_
    {
        typedef Header_<ContainerAllocator> Type;

        Header_() : seq(0), stamp(), frame_id()
        {
        }
        Header_(const ContainerAllocator &_alloc) : seq(0), stamp(), frame_id(_alloc)
        {
            (void)_alloc;
        }



        typedef uint32_t _seq_type;
        _seq_type seq;

        typedef uint64_t _stamp_type;
        _stamp_type stamp;

        typedef std::
                basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
                        _frame_id_type;
        _frame_id_type frame_id;
    };  // struct Header_

    typedef ::std_msgs::Header_<std::allocator<void>> Header;
}  // namespace std_msgs
// NOLINTEND(*)
