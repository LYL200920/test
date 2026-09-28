	.list

	.extern _data
	.extern _mdata
	.extern _ebss
	.extern _bss
	.extern _edata
	.extern _main
	.extern _ustack_start
	.extern _ustack_end
	.extern _istack_start
	.extern _istack_end
	.extern _rvectors_start
	.extern _fvectors_start
	.extern _exit
	.extern __ZN13mcb_v13_board6g_instE
	.extern __ZN13mcb_v13_board37maybe_reset_config_to_factory_defaultEv

	.section .text

.global _PowerON_Reset
	.align 4
_PowerON_Reset:

/* initialise user stack pointer */

	mvtc	#_ustack_end,USP

/* initialise interrupt stack pointer */
	mvtc	#_istack_end,ISP

/* setup intb */
	mvtc	#_fvectors_start,intb
	mvtc	#_rvectors_start,intb

/* setup FPSW */
	mvtc	#100h, fpsw

/* load data section from ROM to RAM */

	mov	#_mdata,r2	// src ROM address of data section in R2
	mov	#_data,r1	// dest start RAM address of data section in R1
	mov	#_edata,r3	// end RAM address of data section in R3
	sub	r1,r3		// size of data section in R3 (R3=R3-R1)
	smovf			// block copy R3 bytes from R2 to R1

/* bss initialisation : zero out bss */

	mov	#00h,r2
	mov	#_ebss, r3	// store the end address of bss in R3
	mov	#_bss,r1
	sub	r1,r3		// size of bss section in R3 (R3=R3-R1)
	sstr.b

#if !defined(GDB_SIMULATOR_DEBUG) && 1
/* call the hardware initialiser */

// setup PSW to use userstack.
// don't enable interrupts yet, as we need to do some IO configuration
// for that first.
	mvtc	#(0 << 16) | (1 << 17), psw

// change PSW PM to user-mode
// can't do that because FCU user-mode code needs to disable interrupts,
// which requires privileged mode.
// could work around this by using a software interrupt/trap ...

/*
	MVFC	PSW,R1
//	OR	#00100000h,R1
	PUSH.L	R1
.Lrtepc0:
	MVFC	PC,R1
	ADD	#(.Lrtepc1 - .Lrtepc0),R1
	PUSH.L	R1
	RTE
	NOP
	NOP
.Lrtepc1:
	NOP
	NOP
*/
#endif

// run static initializers which at first will run the board
// constructor and initialize the hardware.
	bsr.a	__rx_init

// reset to factory default
// as a side effect, this also creates a strong reference to the board instance
// global variable.  if nothing references it, it will get stripped.
	mov.l	#__ZN13mcb_v13_board6g_instE, r1
	bsr.a	__ZN13mcb_v13_board37maybe_reset_config_to_factory_defaultEv

// start user program
	bsr.a	_main

// finish
	bra.a 	_exit


	.global	_rx_run_preinit_array
	.type	_rx_run_preinit_array,@function
_rx_run_preinit_array:
	mov	#__preinit_array_start,r1
	mov	#__preinit_array_end,r2
	bra.a	_rx_run_inilist

	.global	_rx_run_init_array
	.type	_rx_run_init_array,@function
_rx_run_init_array:
	mov	#__init_array_start,r1
	mov	#__init_array_end,r2
	mov	#4, r3
	bra.a	_rx_run_inilist

#ifndef DISABLE_GLOBAL_DTORS
	.global	_rx_run_fini_array
	.type	_rx_run_fini_array,@function
_rx_run_fini_array:
	mov	#__fini_array_start,r2
	mov	#__fini_array_end,r1
	mov	#-4, r3
	/* fall through */
#endif

_rx_run_inilist:
next_inilist:
	cmp	r1,r2
	beq.b	done_inilist
	mov.l	[r1],r4
	cmp	#-1, r4
	beq.b	skip_inilist
	cmp	#0, r4
	beq.b	skip_inilist
	pushm	r1-r3
	jsr	r4
	popm	r1-r3
skip_inilist:
	add	r3,r1
	bra.b	next_inilist
done_inilist:
	rts

// --------------------------------------------------------------------

	.section .init,"ax"
	.balign 4

	.global __rx_init
__rx_init:
	bsr.a	_rx_run_preinit_array
	bra.a	_rx_run_init_array

	.section .fini,"ax"
	.balign 4

// --------------------------------------------------------------------

	.section .init,"ax"
	bsr.a   _rx_run_preinit_array
	bsr.a   _rx_run_init_array
	rts

	.global __rx_init_end
	__rx_init_end:



	.section .fini,"ax"
	.balign 4
	.global __rx_fini
__rx_fini:
#ifndef DISABLE_GLOBAL_DTORS
	bsr.a	_rx_run_fini_array
#else
	rts
#endif
    .global __rx_fini_end
__rx_fini_end:


// --------------------------------------------------------------------
// exit function
	.section .text
.global _exit
.global __exit
_exit:
__exit:
	bsr.a	__rx_fini

_loop_here:
	bra _loop_here


// --------------------------------------------------------------------

        .section .sdata
        .balign 4
        .global __gp
	.weak   __gp
__gp:

	.section .data
	.global ___dso_handle
	.weak   ___dso_handle
___dso_handle:
	.long	0

