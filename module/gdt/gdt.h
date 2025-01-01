#include "../types.h"
#include "../stdio/stdio.h"
#include "../terminal/terminal.h"

#ifndef GDT_H
#define GDT_H

#define GDT_SIZE 5

struct __attribute__((__packed__)) GDTDescriptor;

void set_gdt_entry(struct GDTDescriptor *entry, uint32_t base, uint32_t limit, uint8_t access, uint8_t flags);

void setup_gdt();

struct __attribute__((__packed__)) GDTR;

void load_gdt();

void debug_gdtr();

#endif