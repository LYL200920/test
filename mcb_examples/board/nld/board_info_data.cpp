/*

the board info data always remains in .rodata.  it can and will be overwritten
in the final ROM image before flashing the ROM image during production.

the data block can be anywhere in the ROM image, but the programming tool
needs to find its location.  we use the reserved ISR vector slot -12
(ROM address 0x00000010) to store a pointer to the board info data.
this is a reserved ISR vector slot on the cortex-m0 processor.

the ISR vector table is actually an constexpr array of function pointers,
so the compiler will refuse putting a normal data pointer.

the workaround for this is to use extern "C" for the global variable name
and put the data block into this separate compilation unit.  in the other
compilation unit, where the ISR vector table is constructed, it is referenced
as a function pointer type.  notice that this hack works only when the
translation units are not merged, e.g. by LTO.  it's sufficient to disable
LTO for this translation unit.

*/

#include <type_traits>
#include <board/board_info.hpp>

extern "C" [[gnu::section (".rodata")]]
const std::aligned_storage<sizeof (nld_board_info), alignof (nld_board_info)>::type board_info_data = { };

