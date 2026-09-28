// ---------------------------------------------------------------------------
// to place code into RAM use the following attributes.
// the section attribute is funny ....

#ifdef __ramfunc_attr__
#error

#else
/*
#define __ramfunc_attr__ __attribute__((flatten, noclone, no_icf, used, noinline, \
					section (".data,\"a\",@progbits ;")))
//					          ^^^^^^^^^^^^^^^^^^^^^
*/
//	the data section string will be suffixed by the compiler with
//		,"ax",@progbits
//	it does that for every code section which results in a warning
//	"setting incorrect section attributes..." and also confuses the
//	binutils 'size' program when calculating sizes for .text and .data.
//	the reason is the "x" section attribute.  to avoid it, we specify the
//	attributes ourselves and add a ';'.  this will create a comment in the
//	output assembler code and the compiler generated suffix string will be
//	rendered ineffective.
//
//	it seems that with the "fix" above still another warning might appear
//	(when building with LTO):
//	     Warning: ignoring changed section attributes for .data
//	to "fix" that, just remove all section attributes as below

// RX64M/RX71M MCUs with code flash >= 2.5 MByte can execute code from one
// half while rewriting another half.  unfortunately the user-boot area
// is part of that one half of the memory.  MCUs with 2 MByte flash don't
// support any of thtat.
#ifdef RX_TEXT_IN_RAM
  #define __ramfunc_attr__

#else
  #define __ramfunc_attr__ gnu::flatten, gnu::noinline, gnu::no_icf, gnu::used, \
			   gnu::noinline, gnu::section (".data ;")
#endif

#endif // __ramfunc_attr__

// ---------------------------------------------------------------------------
/*
// on RX there are spurious linker errors when invoking a function in .data
// from a function in .text.
// this is because the function call looks like this:
//    bsr	__ZN3fcu17erase_rom_block_1ENS_11rom_pe_addrE
//
// and the checks in the linker are buggy.  to avoid the problem the
// symbol (function address) can be loaded into a register first via mov.l.
// to force this, we can add an empty asm statement with a register input
// constraint.  however, with this the function wrapper below only supports
// static functions and support for member functions is more complicated,
// also on the caller site.

*/

#ifdef RX_TEXT_IN_RAM
template <typename F, typename... Args> inline auto
invoke_ramfunc (F&& f, Args... args) -> std::result_of_t <F (Args...)>
{
  return f (std::forward<Args> (args)...);
}

#else
template <typename F, typename... Args> inline auto
invoke_ramfunc (F&& f, Args... args) -> std::result_of_t <F (Args...)>
{
//  auto ff = f;
//  asm ("" : : "r" (ff));

  // the only way to really force the operand into the register is by
  // emitting a move insn that explicitly loads the function address into
  // a register.  on RX a mov.L can be used for that.

  std::result_of_t <F (Args...)> (*ff)(Args...);

  #ifdef __RX__
    asm ("mov.L	%1,%0" : "=r" (ff) : "" (f));
  #else
    ff = f;
  #endif

  return (ff (std::forward<Args> (args)...));
}
#endif

/*
as it turned out, when the image is located in the user boot area (0xFF7...)
binutils (it seems) will generate bsr.a instructions on RX with wrong target
addresses.  as an end result, it will not jump into RAM but somewhere else.
to work around that problem we have to force the function address into a
register, which will load the correct value.  then do the function call through
the register via jsr instead of bsr.a.

it seems it's enough to just add an asm statement that references the function
address and forces it into a register.  if it's done immediately before the
function call the optimizers will re-use the value in the register when
emitting the function call insn:

   __force_reg__ (function_name);
   function_name (arg, 123);

however, that certainly depends on the optimization level and is not stable.

#define __force_reg__(x) asm ("" : : "r" (x))
*/

