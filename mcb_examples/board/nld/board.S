	.syntax unified
	.cpu cortex-m0
	.fpu softvfp
	.thumb

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
	.extern _ZN9nld_board6g_instE

.global PowerON_Reset

	.section .text,"ax",%progbits

	.align 4
	.thumb_func
	.type PowerON_Reset,%function
PowerON_Reset:

	// disable interrupts
	cpsid	i

	// after reset it is in "thread mode" and uses the main stack (MSP).
	// the MSP value has been loaded from the ISR vector table
	// automatically, which is now set to the istack_end.

	// switch current stack (SP) to PSP
	movs	r0,#2
	msr	control,r0

	// set the SP/PSP to ustack_end
	ldr	r0,=_ustack_end
	mov	sp,r0

	// enable SYSCFG & COMP clock in RCC_APB2ENR (0x40021018)
#ifndef BOARD_NO_RESET_HW_INIT
	ldr	r0,=0x40021018
	movs	r1,#1
	str	r1,[r0]
#endif

	// load data section from ROM to RAM
	ldr	r0,=_mdata	// copy src address in ROM (32 byte aligned)
	ldr	r1,=_data	// copy dst address in RAM (32 byte aligned)
	ldr	r2,=_edata	// end dst address in RAM (32 byte aligned)
	b	.Ldata_copy_init

.Ldata_copy:
	ldm	r0!,{r3,r4,r5,r6}	// read 16 bytes
	stm	r1!,{r3,r4,r5,r6}	// write 16 bytes
.Ldata_copy_init:
	cmp	r1,r2
	bne	.Ldata_copy


	// zero out bss
	ldr	r0,=_bss	// dst start address in RAM (32 byte alinged)
	ldr	r1,=_ebss	// dst end address in RAM (32 byte aligned)
	movs	r3,#0
	movs	r4,#0
	movs	r5,#0
	movs	r6,#0
	b	.Lbss_init

.Lbss_fill:
	stm	r0!,{r3,r4,r5,r6}	// write 16 bytes
.Lbss_init:
	cmp	r0,r1
	bne	.Lbss_fill

	// enable interrupts
	cpsie	i

	// run static initializers which at first will run the board
	// constructor and initialize the hardware.
	ldr	r4,=__preinit_array_start
	ldr	r5,=__preinit_array_end
	movs	r6,#4
	bl	_run_initlist

	ldr	r4,=__init_array_start
	ldr	r5,=__init_array_end
	movs	r6,#4
	bl	_run_initlist

	// start user program
	bl	main

	// finish
.global exit
.global _exit
.global __exit
	.thumb_func
exit:
_exit:
__exit:

#ifndef DISABLE_GLOBAL_DTORS
	ldr	r4,=__fini_array_end
	ldr	r5,=__fini_array_start
	movs	r6,#-4
	bl	_run_initlist
#endif

.Lloop_exit:
	cpsid	i
	wfi
	b	.Lloop_exit


// --------------------------------------------------
// helper function to invoke init/finish functions
// r4 = array address start
// r5 = array address end
// r6 = step direction (4 or -4)
	.thumb_func
	.type _run_initlist,%function
_run_initlist:
	push	{lr}

.Linitlist_next:
	cmp	r4,r5
	beq	.Linitlist_done

	ldr	r0,[r4]

	movs	r1,#0			// skip entry if it is 0x00000000
	cmp	r0,r1
	beq	.Linitlist_skipentry

	mvns	r1,r1			// skip entry if it is 0xFFFFFFFF
	cmp	r0,r1
	beq	.Linitlist_skipentry

	blx	r0

.Linitlist_skipentry:
	add	r4,r6
	b	.Linitlist_next

.Linitlist_done:
	pop	{pc}


// --------------------------------------------------------------------

	// this creates a strong reference to the board instance
	// global variable.  if nothing references it, it might get stripped.
	.align 4
	.word	_ZN9nld_board6g_instE

// --------------------------------------------------------------------


	.section .sdata
	.balign 4
	.global _gp
	.weak _gp
_gp:
	.section .data
	.global __dso_handle
	.weak   __dso_handle
__dso_handle:
	.word	0

