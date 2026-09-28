
OUTPUT_FORMAT("elf32-rl78", "elf32-rl78","elf32-rl78")
OUTPUT_ARCH(rl78)

ENTRY(_PowerON_Reset)
SEARCH_DIR("/usr/local/rl78-elf/lib");


/*
some values are provided by the cmake toolchain generated linker script
snippet, from which we can calculate everything else.

_rl78_rom_size = 1024*1;
_rl78_ram_size = 128;
_rl78_stack_size = 64;
_rl78_rom_mirror_start = 0xF8000;

*/

INCLUDE memory_config.ld

_rl78_fvectors_size = 128;
_rl78_callt_table_size = 64;
_rl78_option_bytes_size = 4;
_rl78_security_id_bytes_size = 10;

_rl78_fvectors_start = 0x00000;
_rl78_callt_table_start = _rl78_fvectors_start + _rl78_fvectors_size;
_rl78_option_bytes_start = _rl78_callt_table_start + _rl78_callt_table_size;
_rl78_security_id_bytes_start = _rl78_option_bytes_start + _rl78_option_bytes_size;

_rl78_rom_start = _rl78_security_id_bytes_start + _rl78_security_id_bytes_size;

_rl78_ram_end = 0xFFEE0;
_rl78_ram_start = _rl78_ram_end - _rl78_ram_size;

_rl78_stack_end = _rl78_ram_end;
_rl78_stack_start = _rl78_stack_end - _rl78_stack_size;

_rl78_user_ram_end = _rl78_ram_end - _rl78_stack_size;

MEMORY
{
  rom (rx) : ORIGIN = _rl78_rom_start, LENGTH = _rl78_rom_size - _rl78_rom_start
  rom_mirror (rx): ORIGIN = _rl78_rom_start + _rl78_rom_mirror_start, LENGTH = _rl78_rom_size - _rl78_rom_start
  ram (rwx) : ORIGIN = _rl78_ram_start, LENGTH = _rl78_user_ram_end - _rl78_ram_start
}

SECTIONS
{
  /* Read-only sections, merged into text segment: */

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
  .fvectors (_rl78_fvectors_start) : AT (_rl78_fvectors_start)
  {
    _fvectors_start = .;
    KEEP(*(.fvectors))
  }

  .callt_table (_rl78_callt_table_start) : AT (_rl78_callt_table_start)
  {
    _callt_table_start = .;
    KEEP(*(.callt_table))
  }

  .option_bytes (_rl78_option_bytes_start) : AT (_rl78_option_bytes_start)
  {
    _option_bytes_start = .;
    KEEP(*(.option_bytes))
  }

  .security_id (_rl78_security_id_bytes_start) : AT (_rl78_security_id_bytes_start)
  {
    _security_id_start = .;
    KEEP (*(.security_id))
  }

  /* .text and .rodata (and similar) sections reside in ROM, but when accessing
    ROM as data with the RL78 CPU, it needs to use either es:[..] addressing
    and  load es accordingly (with 0), or it can be accessed through the rom
    mirror at an 0xF.... address and accessed in the same way as RAM.
    so we keep the actual ROM address as LMA but adjust the VMA into the
    ROM mirror.  */

  .text :
  {
    __version_tag_start = (. + _rl78_rom_mirror_start);
    KEEP(*(.version_tag))
    __version_tag_end = (. + _rl78_rom_mirror_start);

    /* the fvectors table consits of 16 bit entries.  this means that all
       the isr functions must be located in the lower 64K address space.  */
    *(.text.isr .text.isr.*)

    *(.text.unlikely .text.*_unlikely .text.unlikely.*)
    *(.text.exit .text.exit.*)
    *(.text.startup .text.startup.*)
    *(.text.hot .text.hot.*)
    *(.text .stub .text.* .gnu.linkonce.t.*)
    /* .gnu.warning sections are handled specially by elf32.em.  */
    *(.gnu.warning)
  } > rom

  .init :
  {
    KEEP (*(SORT_NONE(.init)))
  }
  .fini :
  {
    KEEP (*(SORT_NONE(.fini)))
  }

  .plt : { *(.plt) }
  .iplt : { *(.iplt) }

  PROVIDE (__etext = .);
  PROVIDE (_etext = .);
  PROVIDE (etext = .);

  .rodata  : { *(.rodata .rodata.* .gnu.linkonce.r.*) } > rom_mirror AT> rom
  .rodata1 : { *(.rodata1) } > rom_mirror AT> rom

   /* FIXME: always have to specify the VMA and LMA for .rodata like sections,
      which can be accessed from CPU as data.
      on RL78 devices with more flash, some data can go in the higher addresses,
      like usually not used exception tables etc.  however, then the code
      that accesses the data must use __far pointers, which is not available
      for C++ ...
      for now, map everything into the ROM mirror area.  */

  .eh_frame_hdr : { *(.eh_frame_hdr) *(.eh_frame_entry .eh_frame_entry.*) }  > rom_mirror AT> rom
  .eh_frame       : ONLY_IF_RO { KEEP (*(.eh_frame)) *(.eh_frame.*) }  > rom_mirror AT> rom
  .gcc_except_table   : ONLY_IF_RO { *(.gcc_except_table
  .gcc_except_table.*) }  > rom_mirror AT> rom
  .gnu_extab   : ONLY_IF_RO { *(.gnu_extab*) }  > rom_mirror AT> rom
  /* These sections are generated by the Sun/Oracle C++ compiler.  */
  .exception_ranges   : ONLY_IF_RO { *(.exception_ranges
  .exception_ranges*) }  > rom_mirror AT> rom
  /* Adjust the address for the data segment.  We want to adjust up to
     the same address within the page on the next page up.
  . = ALIGN(256) + (. & (256 - 1));  */
  /* Exception handling  */
  .eh_frame       : ONLY_IF_RW { KEEP (*(.eh_frame)) *(.eh_frame.*) } > rom_mirror AT> rom
  .gnu_extab      : ONLY_IF_RW { *(.gnu_extab) } > rom_mirror AT> rom
  .gcc_except_table   : ONLY_IF_RW { *(.gcc_except_table .gcc_except_table.*) } > rom_mirror AT> rom
  .exception_ranges   : ONLY_IF_RW { *(.exception_ranges .exception_ranges*) } > rom_mirror AT> rom

  /* .ctors and .dtors is the array of function pointers to global constructors
     and global destructors.  on RL78 the addresses will be stored as 16 bit
     and thus the functions must be located in the lower 64K of the flash.  */
  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > rom_mirror AT> rom

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array ))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > rom_mirror AT> rom

  .ctors :
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
  } > rom_mirror AT> rom

/* this is conditionally generated by the cmake toolchain file and included
   in this linker script.  if global destructors are disabled the included
   snippet will be empty.

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array ))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > rom_mirror AT> rom

  .dtors :
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  } > rom_mirror AT> rom
*/

  INCLUDE global_dtors.ld


  .got            : { *(.got.plt) *(.igot.plt) *(.got) *(.igot) }

  .jcr            : { KEEP (*(.jcr)) }
  .data.rel.ro : { *(.data.rel.ro.local* .gnu.linkonce.d.rel.ro.local.*) *(.data.rel.ro .data.rel.ro.* .gnu.linkonce.d.rel.ro.*) }
  .dynamic        : { *(.dynamic) }

  /* the initialized data is located in the ROM.  when copying the data
     from ROM into RAM, need to use the ROM mirror area.
     _mdata is only used in the init code as a copy start address.  */
  .mdata : 
  {
    _mdata = .;
  } > rom_mirror AT> rom

  .data :
  {
    _data = .;
    *(.data .data.* .gnu.linkonce.d.*)
    *(.data1)

    /* Thread Local Storage sections  */
    *(.tdata .tdata.* .gnu.linkonce.td.*)

    *(.sdata2 .sdata2.* .gnu.linkonce.s2.*)

    /* We want the small data sections together, so single-instruction offsets
       can access them all, and initialized data all before uninitialized, so
       we can shorten the on-disk segment size.  */
    *(.sdata .sdata.* .gnu.linkonce.s.*)

    SORT(CONSTRUCTORS)
  } > ram AT> rom

/*  . = ALIGN(32); */
  _edata = .; PROVIDE (edata = .);

/*  . = ALIGN(32); */
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

/*  . = ALIGN(32); */
  _end = .; PROVIDE (_ebss = .);


/*  . = ALIGN (32); */
  .ram_start :
  {
    __ram_start = .;
    PROVIDE_HIDDEN (___malloc_start = .);
  }
  .ram_end (_rl78_user_ram_end) : AT (_rl78_user_ram_end)
  {
    __ram_end = .;
    PROVIDE_HIDDEN (___malloc_end = .);
  }
  .stack (_rl78_stack_end) : AT (_rl78_stack_end)
  {
    _stack = .;
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

  /* FIXME: not sure what this is for ... */
  PROVIDE (__rl78_abs__ = 0);

  /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) }
}

