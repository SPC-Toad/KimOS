#include "module/multiboot.h"
#include "module/elf.h"
#include "module/types.h"
#include "module/terminal/terminal.h"
#include "module/stdio/stdio.h"
#include "module/gdt/gdt.h"

void init( multiboot* pmb ) {
    terminal_initialize();
    terminal_writestring("terminal test\n");
    terminal_writestring("exec setup GDT\n");
    setup_gdt();
    terminal_writestring("GDT setup passed.\n Entering loading GDT\n");
    load_gdt();
    terminal_writestring("Loading GDT Successful\n");
    debug_gdtr();
    terminal_writestring("\n");


    terminal_writestring("-------- Test Success --------\n");
    terminal_writestring("Reached End. Halting\n");
    while (1);
}