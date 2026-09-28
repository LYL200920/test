/* Script for -z combreloc: combine and sort reloc sections */
/* Copyright (C) 2014-2015 Free Software Foundation, Inc.
   Copying and distribution of this script, with or without modification,
   are permitted in any medium without royalty provided the copyright
   notice and this notice are preserved.  */

/* The output format is specified by the --oformat= option which is defined
   in the cmake toolchain file.  */
/*OUTPUT_FORMAT("elf32-rx-be", "elf32-rx-be", "elf32-rx-be") */
/*OUTPUT_FORMAT("elf32-rx-le", "elf32-rx-le", "elf32-rx-le") */

OUTPUT_ARCH(rx)
ENTRY(_PowerON_Reset)

/*
MEMORY
{
  RAM (w) : ORIGIN = 0x00000100, LENGTH = 1024*128 - 256
  ROM (w) : ORIGIN = 0xFFFC0000, LENGTH = 1024*768 - 128
}
*/

/*
some values are provided by the cmake toolchain generated linker script
snippet, from which we can calculate everything else.

varying the rom end address allows creating normal user flash and user boot
flash images.

_rx_rom_end = 0xFFFFFFFF;
_rx_rom_size = 1024*4;
_rx_ram_start = 0x00000100;
_rx_ram_size = 128*1024;
_rx_istack_size = 1024;
_rx_ustack_size = 2048;


the included file is expected to be in the build directory of the "board" library.
*/

INCLUDE board/memory_config.ld

_rx_fvectors_size = 128;
_rx_fvectors_start = (_rx_rom_end - _rx_fvectors_size) + 1;
_rx_rom_start = (_rx_rom_end - _rx_rom_size) + 1;

_rx_ram_end = _rx_ram_start + _rx_ram_size;

_rx_istack_end = _rx_ram_end;
_rx_istack_start = _rx_istack_end - _rx_istack_size;

_rx_ustack_end = _rx_istack_start;
_rx_ustack_start = _rx_ustack_end - _rx_ustack_size;

_rx_user_ram_end = _rx_ram_end - _rx_istack_size - _rx_ustack_size;


SECTIONS
{

  .fvectors (_rx_fvectors_start) : AT (_rx_fvectors_start)
  {
    _fvectors_start = .;
    KEEP(*(.fvectors))
  }

  /* Read-only sections, merged into text segment: */
/*
  PROVIDE (__executable_start = SEGMENT_START("text-segment", 0x10000000));
  . = SEGMENT_START("text-segment", 0x10000000);
*/


/*
  .interp         : { *(.interp) }
  .note.gnu.build-id : { *(.note.gnu.build-id) }
  .hash           : { *(.hash) }
  .gnu.hash       : { *(.gnu.hash) }
  .dynsym         : { *(.dynsym) }
  .dynstr         : { *(.dynstr) }
  .gnu.version    : { *(.gnu.version) }
  .gnu.version_d  : { *(.gnu.version_d) }
  .gnu.version_r  : { *(.gnu.version_r) }
  .rel.dyn        :
    {
      *(.rel.init)
      *(.rel.text .rel.text.* .rel.gnu.linkonce.t.*)
      *(.rel.fini)
      *(.rel.rodata .rel.rodata.* .rel.gnu.linkonce.r.*)
      *(.rel.data.rel.ro .rel.data.rel.ro.* .rel.gnu.linkonce.d.rel.ro.*)
      *(.rel.data .rel.data.* .rel.gnu.linkonce.d.*)
      *(.rel.tdata .rel.tdata.* .rel.gnu.linkonce.td.*)
      *(.rel.tbss .rel.tbss.* .rel.gnu.linkonce.tb.*)
      *(.rel.ctors)
      *(.rel.dtors)
      *(.rel.got)
      *(.rel.sdata .rel.sdata.* .rel.gnu.linkonce.s.*)
      *(.rel.sbss .rel.sbss.* .rel.gnu.linkonce.sb.*)
      *(.rel.sdata2 .rel.sdata2.* .rel.gnu.linkonce.s2.*)
      *(.rel.sbss2 .rel.sbss2.* .rel.gnu.linkonce.sb2.*)
      *(.rel.bss .rel.bss.* .rel.gnu.linkonce.b.*)
      PROVIDE_HIDDEN (__rel_iplt_start = .);
      *(.rel.iplt)
      PROVIDE_HIDDEN (__rel_iplt_end = .);
    }
  .rela.dyn       :
    {
      *(.rela.init)
      *(.rela.text .rela.text.* .rela.gnu.linkonce.t.*)
      *(.rela.fini)
      *(.rela.rodata .rela.rodata.* .rela.gnu.linkonce.r.*)
      *(.rela.data .rela.data.* .rela.gnu.linkonce.d.*)
      *(.rela.tdata .rela.tdata.* .rela.gnu.linkonce.td.*)
      *(.rela.tbss .rela.tbss.* .rela.gnu.linkonce.tb.*)
      *(.rela.ctors)
      *(.rela.dtors)
      *(.rela.got)
      *(.rela.sdata .rela.sdata.* .rela.gnu.linkonce.s.*)
      *(.rela.sbss .rela.sbss.* .rela.gnu.linkonce.sb.*)
      *(.rela.sdata2 .rela.sdata2.* .rela.gnu.linkonce.s2.*)
      *(.rela.sbss2 .rela.sbss2.* .rela.gnu.linkonce.sb2.*)
      *(.rela.bss .rela.bss.* .rela.gnu.linkonce.b.*)
      PROVIDE_HIDDEN (__rela_iplt_start = .);
      *(.rela.iplt)
      PROVIDE_HIDDEN (__rela_iplt_end = .);
    }
  .rel.plt        :
    {
      *(.rel.plt)
    }
  .rela.plt       :
    {
      *(.rela.plt)
    }
*/
  .text (_rx_rom_start): AT (_rx_rom_start)
  {
    __text_start = .;

    __version_tag_start = .;
    KEEP(*(.version_tag))
    __version_tag_end = .;

    *(.text.unlikely .text.*_unlikely .text.unlikely.*)
    *(.text.exit .text.exit.*)
    *(.text.startup .text.startup.*)
    *(.text.hot .text.hot.*)
    *(.text .stub .text.* .gnu.linkonce.t.*)
    /* .gnu.warning sections are handled specially by elf32.em.  */
    *(.gnu.warning)
    *(P)
  }

  .init :
  {
    KEEP (*(SORT_NONE(.init)))
  }
  .plt : { *(.plt) }
  .iplt : { *(.iplt) }

  .fini :
  {
    KEEP (*(SORT_NONE(.fini)))
  }

  /* relocatable vector table  */
  .rvectors :
  {
    _rvectors_start = .;
    KEEP(*(.rvectors))
    _rvectors_end = .;
  }

  PROVIDE (__etext = .);
  PROVIDE (_etext = .);
  PROVIDE (etext = .);

  .rodata         : { *(.rodata .rodata.* .gnu.linkonce.r.*) *(C_1) *(C_2) *(C) }
  .rodata1        : { *(.rodata1) }

  .eh_frame_hdr : { *(.eh_frame_hdr) *(.eh_frame_entry .eh_frame_entry.*) }
  .eh_frame       : ONLY_IF_RO { KEEP (*(.eh_frame)) *(.eh_frame.*) }
  .gcc_except_table   : ONLY_IF_RO { *(.gcc_except_table
  .gcc_except_table.*) }
  .gnu_extab   : ONLY_IF_RO { *(.gnu_extab*) }
  /* These sections are generated by the Sun/Oracle C++ compiler.  */
  .exception_ranges   : ONLY_IF_RO { *(.exception_ranges
  .exception_ranges*) }
  /* Adjust the address for the data segment.  We want to adjust up to
     the same address within the page on the next page up.  */
  . = ALIGN(256) + (. & (256 - 1));
  /* Exception handling  */
  .eh_frame       : ONLY_IF_RW { KEEP (*(.eh_frame)) *(.eh_frame.*) }
  .gnu_extab      : ONLY_IF_RW { *(.gnu_extab) }
  .gcc_except_table   : ONLY_IF_RW { *(.gcc_except_table .gcc_except_table.*) }
  .exception_ranges   : ONLY_IF_RW { *(.exception_ranges .exception_ranges*) }

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  }
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array ))
    PROVIDE_HIDDEN (__init_array_end = .);
  }

  .ctors          :
  {
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
  }

/* this is conditionally generated by the cmake toolchain file and included
   in this linker script.  if global destructors are disabled the included
   snippet will be empty.
   the file is expected to be in the build directory of the "board" library.

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array ))
    PROVIDE_HIDDEN (__fini_array_end = .);
  }

  .dtors :
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  }

*/
  INCLUDE board/global_dtors.ld


  .got            : { *(.got.plt) *(.igot.plt) *(.got) *(.igot) }

  .jcr            : { KEEP (*(.jcr)) }
  .data.rel.ro : { *(.data.rel.ro.local* .gnu.linkonce.d.rel.ro.local.*) *(.data.rel.ro .data.rel.ro.* .gnu.linkonce.d.rel.ro.*) }
  .dynamic        : { *(.dynamic) }

  .text_end :
  {
    __text_end = .;

    . = ALIGN(32);
    _mdata = .;
  }

  .data _rx_ram_start : AT (_mdata)
  {
    _data = .;
    *(.data .data.* .gnu.linkonce.d.*)
    *(.data1)
    *(D_1) *(D_2) *(D)

    /* Thread Local Storage sections  */
    *(.tdata .tdata.* .gnu.linkonce.td.*)

    *(.sdata2 .sdata2.* .gnu.linkonce.s2.*)

    /* We want the small data sections together, so single-instruction offsets
       can access them all, and initialized data all before uninitialized, so
       we can shorten the on-disk segment size.  */
    *(.sdata .sdata.* .gnu.linkonce.s.*)

    SORT(CONSTRUCTORS)
  }

  . = ALIGN(32);
  _edata = .; PROVIDE (edata = .);

  . = ALIGN(32);
  __bss_start = .; PROVIDE (_bss = .);
  .sbss :
  {
    *(.dynsbss)
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)
  }

  .bss :
  {
    *(.dynbss)
    *(.bss .bss.* .gnu.linkonce.b.*)
    *(B_1) *(B_2) *(B)
    *(COMMON)
    /* Align here to ensure that the .bss section occupies space up to
       _end.  Align after .bss to ensure correct alignment even if the
       .bss section disappears because there are no input sections.
       FIXME: Why do we need it? When there is no .bss section, we don't
       pad the .data section.
     . = ALIGN(. != 0 ? 32 / 8 : 1);
      */

    /* Thread Local Storage sections  */
    *(.tbss .tbss.* .gnu.linkonce.tb.*) *(.tcommon)

    *(.sbss2 .sbss2.* .gnu.linkonce.sb2.*)
  }

  . = ALIGN(32);
  _end = .; PROVIDE (end = .); PROVIDE (_ebss = .);


  . = ALIGN (32);
  .ram_start :
  {
    __ram_start = .;
    PROVIDE_HIDDEN (___malloc_start = .);
  }
  .ram_end (_rx_user_ram_end) : AT (_rx_user_ram_end)
  {
    __ram_end = .;
    PROVIDE_HIDDEN (___malloc_end = .);
  }

  .ustack (_rx_ustack_start) : AT (_rx_ustack_start)
  {
    _ustack_start = .;
    . = . + _rx_ustack_size;
    _ustack_end = .;
  }

  .istack (_rx_istack_start) : AT (_rx_istack_start)
  {
    _istack_start = .;
    . = . + _rx_istack_size;
    _istack_end = .;
  }


  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line .debug_line.* .debug_line_end ) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  /* DWARF 3 */
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  .debug_ranges   0 : { *(.debug_ranges) }
  /* DWARF Extension.  */
  .debug_macro    0 : { *(.debug_macro) }
  .gnu.attributes 0 : { KEEP (*(.gnu.attributes)) }

/*
    .stack         (DEFINED(__stack) ? __stack : 0xbffffffc) :
  {
    __stack = .;
    *(.stack)
    LONG(0xdeaddead)
  }
*/


  /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) }
}

