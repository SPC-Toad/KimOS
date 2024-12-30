OUTPUT_FORMAT("elf32-i386")
OUTPUT_ARCH(i386)
ENTRY(_start)
SECTIONS {
  /* Bootstrap physical section, at virtual and physical address 0x100000 */
  . = 0x100000;

  . = ALIGN(0x1000);
  .text : { *(.multiboot .text) }

  . = ALIGN(0x1000);
  .rodata : { *(.rodata) }

  . = ALIGN(0x1000);
  .data : { *(.data) }

  . = ALIGN(0x1000);
  .bss : { *(.bss COMMON) }

  . = ALIGN(0x1000);
  .eh_frame (NOLOAD) : { *(.eh_frame) } : NONE
}
