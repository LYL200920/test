/* Script for -z combreloc: combine and sort reloc sections */
/* Copyright (C) 2014-2015 Free Software Foundation, Inc.
   Copying and distribution of this script, with or without modification,
   are permitted in any medium without royalty provided the copyright
   notice and this notice are preserved.  */

/* The output format is specified by the --oformat= option which is defined
   in the cmake toolchain file.  */
OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm",
	      "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(PowerON_Reset)

/*
some values are provided by the cmake toolchain generated linker script
snippet, from which we can calculate everything else.

varying the rom end address allows creating normal user flash and user boot
flash images.

_arm_rom_start = 0x00000000;
_arm_rom_size = 1024*4;
_arm_ram_start = 0x00000100;
_arm_ram_size = 128*1024;
_arm_istack_size = 1024;
_arm_ustack_size = 2048;

the included file is expected to be in the build directory of the "board" library.
*/

INCLUDE board/memory_config.ld

_arm_rom_end = _arm_rom_start + _arm_rom_size;
_arm_ram_end = _arm_ram_start + (_arm_ram_size - _arm_istack_size - _arm_ustack_size);
_arm_ustack_start = _arm_ram_end;
_arm_istack_start = _arm_ram_end + _arm_ustack_size;


MEMORY
{
  ROM (rx) : ORIGIN = _arm_rom_start, LENGTH = _arm_rom_size
  RAM (!rx) : ORIGIN = _arm_ram_start, LENGTH = _arm_ram_end - _arm_ram_start
}

SECTIONS
{

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

  .text (_arm_rom_start): AT (_arm_rom_start)
  {
    PROVIDE (_text_start = .);

    /* fixed reset vector table */
    _fvectors_start = .;
    KEEP(*(.fvectors))
    _fvectors_end = .;

    _version_tag_start = .;
    KEEP(*(.version_tag))
    _version_tag_end = .;

    *(.text.unlikely .text.*_unlikely .text.unlikely.*)
    *(.text.exit .text.exit.*)
    *(.text.startup .text.startup.*)
    *(.text.hot .text.hot.*)
    *(.text .stub .text.* .gnu.linkonce.t.*)
    /* .gnu.warning sections are handled specially by elf32.em.  */
    *(.gnu.warning)
  } > ROM

  .init :
  {
    KEEP (*(SORT_NONE(.init)))
  }  > ROM
  .plt : { *(.plt) } > ROM
  .iplt : { *(.iplt) } > ROM

  .fini :
  {
    KEEP (*(SORT_NONE(.fini)))
  } > ROM

  PROVIDE (__etext = .);
  PROVIDE (_etext = .);
  PROVIDE (etext = .);

  .rodata         : { *(.rodata .rodata.* .gnu.linkonce.r.*) } > ROM
  .rodata1        : { *(.rodata1) } > ROM

  .eh_frame_hdr : { *(.eh_frame_hdr) *(.eh_frame_entry .eh_frame_entry.*) } > ROM
  .eh_frame       : ONLY_IF_RO { KEEP (*(.eh_frame)) *(.eh_frame.*) } > ROM
  .gcc_except_table   : ONLY_IF_RO { *(.gcc_except_table  .gcc_except_table.*) } > ROM
  .gnu_extab   : ONLY_IF_RO { *(.gnu_extab*) }
  .exception_ranges   : ONLY_IF_RO { *(.exception_ranges .exception_ranges*) } > ROM

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > ROM
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array ))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > ROM

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
  } > ROM

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
  } > ROM

  .dtors :
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  } > ROM

*/
  INCLUDE board/global_dtors.ld


  .got            : { *(.got.plt) *(.igot.plt) *(.got) *(.igot) } > ROM

  .jcr            : { KEEP (*(.jcr)) } > ROM
  .data.rel.ro : { *(.data.rel.ro.local* .gnu.linkonce.d.rel.ro.local.*) *(.data.rel.ro .data.rel.ro.* .gnu.linkonce.d.rel.ro.*) } > ROM
  .dynamic        : { *(.dynamic) } > ROM

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > ROM

  .text_end :
  {
    __text_end = .;

    . = ALIGN(32);
  } > ROM

  .data _arm_ram_start :
  {
    _mdata = LOADADDR (.data);
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
    . = ALIGN(4);

    . = ALIGN(32);
    _edata = .; PROVIDE (edata = .);
  } > RAM AT > ROM

  /* Exception handling data that needs to be writable -- actually not sure about this one  */
  .eh_frame       : ONLY_IF_RW { KEEP (*(.eh_frame)) *(.eh_frame.*) } > RAM AT > ROM
  .gnu_extab      : ONLY_IF_RW { *(.gnu_extab) } > RAM AT > ROM
  .gcc_except_table   : ONLY_IF_RW { *(.gcc_except_table .gcc_except_table.*) } > RAM AT > ROM
  .exception_ranges   : ONLY_IF_RW { *(.exception_ranges .exception_ranges*) } > RAM AT > ROM

  .rom_end :
  {
    PROVIDE (__rom_end = .);
  } > ROM

  .bss :
  {
    . = ALIGN(32);
    __bss_start = .; PROVIDE (_bss = .);

    *(.dynsbss)
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)

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

   . = ALIGN(. != 0 ? 32 / 8 : 1);
  } > RAM

  . = ALIGN(32);
  _end = .; PROVIDE (end = .); PROVIDE (_ebss = .);


  . = ALIGN (32);
  .ram_start :
  {
    _ram_start = .;
    PROVIDE_HIDDEN (__malloc_start = .);
  }
  .ram_end (_arm_ram_end) : AT (_arm_ram_end)
  {
    _ram_end = .;
    PROVIDE_HIDDEN (__malloc_end = .);
  }

  .ustack (_arm_ustack_start) : AT (_arm_ustack_start)
  {
    _ustack_start = .;
    . = . + _arm_ustack_size;
    _ustack_end = .;
  }

  .istack (_arm_istack_start) : AT (_arm_istack_start)
  {
    _istack_start = .;
    . = . + _arm_istack_size;
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

