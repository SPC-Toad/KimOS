#include "gdt.h"

/* 
    This is the Structure/format of the GDT Descriptor.
    Attribute packed is from the following link:
    https://riptutorial.com/c/example/31059/packing-structures
*/
struct __attribute__((__packed__)) GDTDescriptor {
    uint16_t limit_low;     // Segment limit    ( 0-15)
    uint16_t base_low;      // Base address     (16-31)
    uint8_t base_middle;    // Base address     (32-39)
    uint8_t access_byte;    // Access byte      (40-47)
    uint8_t flags_limit;    // High 4 bits of limit and flags (48-51)
    uint8_t base_high;      // Base address     (56-63)
};

/* 
  This takes the pointer to the GDT Descriptor array index, and assignes the values to the non-linear memory 
  Reference: https://wiki.osdev.org/GDT_Tutorial --> Filling the Table
*/
void set_gdt_entry(struct GDTDescriptor *entry, uint32_t base, uint32_t limit, uint8_t access, uint8_t flags) {
    // Set the limit (split into low and high parts)
    entry->limit_low = limit & 0xFFFF;             // Low 16 bits of the limit
    entry->flags_limit = (limit >> 16) & 0x0F;     // High 4 bits of the limit

    // Set the base address (split into low, middle, and high parts)
    entry->base_low = base & 0xFFFF;               // Low 16 bits of the base
    entry->base_middle = (base >> 16) & 0xFF;      // Middle 8 bits of the base
    entry->base_high = (base >> 24) & 0xFF;        // High 8 bits of the base

    // Set the access byte and flags
    entry->access_byte = access;
    entry->flags_limit |= (flags << 4);            // Set the flags in the upper nibble
}

// We have the first 3 segment descriptors. (NULL, kernel cs and ds)
struct GDTDescriptor gdt[GDT_SIZE];

void setup_gdt() {
    // Null descriptor (required by the CPU)
    set_gdt_entry(&gdt[0], 0, 0, 0, 0);

    // Kernel Mode Code Segment
    set_gdt_entry(&gdt[1], 0x00000000, 0xFFFFF, 0x9A, 0x0C);

    // Kernel Mode Data Segment
    set_gdt_entry(&gdt[2], 0x00000000, 0xFFFFF, 0x92, 0x0C);

    // User Mode Code Segment
    set_gdt_entry(&gdt[3], 0x00000000, 0xFFFFF, 0xFA, 0x0C);

    // User Mode Data Segment
    set_gdt_entry(&gdt[4], 0x00000000, 0xFFFFF, 0xF2, 0x0C);

    // Task State Segment (not needed yet) 
    // set_gdt_entry(&gdt[5], 0x00000000, 0xFFFFF, 0x89, 0x00);
}

/*
  The GDT is pointed to by the value in the GDTR register. 
  This is loaded using the LGDT assembly instruction, 
  whose argument is a pointer to a GDT

  Size: The size of the table in bytes subtracted by 1. 
        This subtraction occurs because the maximum value of Size is 65535, 
        while the GDT can be up to 65536 bytes in length (8192 entries).
        Further, no GDT can have a size of 0 bytes.

  Offset: The linear address of the GDT (not the physical address, paging applies).

  Source: https://wiki.osdev.org/Global_Descriptor_Table --> GDTR
*/
struct __attribute__((__packed__)) GDTR {
    uint16_t limit;  // Size of the GDT (in bytes) - 1
    uint32_t base;   // Base address of the GDT
};

struct GDTR gdtr;

void load_gdt() {
    gdtr.limit = sizeof(gdt) - 1;        // Set the limit to the size of the GDT minus 1
    gdtr.base = (uint32_t)&gdt;          // Set the base to the address of the GDT array

  __asm__ volatile("lgdt %0" : : "m"(gdtr));

}

void debug_gdtr() {
    char base_str[20], limit_str[10];
    itoa(base_str, 'x', gdtr.base);
    itoa(limit_str, 'x', gdtr.limit);
    terminal_writestring("GDTR Base: ");
    terminal_writestring(base_str);
    terminal_writestring("\nGDTR Limit: ");
    terminal_writestring(limit_str);
    terminal_writestring("\n");
}