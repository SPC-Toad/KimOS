/*
 * boot.S for discos
 *
 */
	.globl stack
	
	.bss
	.align 0x1000
	.comm stack, 0x1000 /* setup 4KB stack area in bss */

	.data
	
	.text
	
	.globl _start

_start:
	jmp real_start

	/* Multiboot header -- Safe to place this header in 1st page for GRUB */
	.align 4
	.long 0x1BADB002 /* Multiboot magic number */
	.long 0x00000003 /* Align modules to 4KB, req. mem size */
			 /* See 'info multiboot' for further info */
	.long 0xE4524FFB /* Checksum */

real_start:
	
	/* set up stack */
	movl $stack+0x1000, %esp /* setup 4Kbyte stack */

    /* Call setup_gdt and load_gdt */
    call setup_gdt
    call load_gdt

	/* Reload segment registers */
    mov $0x10, %ax           /* Data segment selector (index 2 * 8 = 0x10) */
    mov %ax, %ds
    mov %ax, %es
    mov %ax, %fs
    mov %ax, %gs
    mov %ax, %ss

	/* Far jump to reload CS */
    ljmp $0x08, $flush_cs    /* Code segment selector (index 1 * 8 = 0x08) */

flush_cs:
	/* save multiboot parameter, for eventual call to C code */
	pushl %ebx

	call init /* start of C code */

	/* In case we return from the call, we want to suspend the processor */
	
	cli
	hlt
loop:
	jmp loop
