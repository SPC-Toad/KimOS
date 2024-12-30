#include "multiboot.h"
#include "types.h"
#define NULL ((void *)0)
int tokenize(char *pathname, char token[10][14]);
void strncpy(char *dest, const char *src);
#define crBegin static int state=0; switch(state) { case 0:

/*
 * ANSI C provides the __LINE__ macro to return the current source line number
 * We can use the line number as a simple form of programmatic state!
 */

#define crReturn(x) do { \
    terminal_writestring("Returning at line: "); \
    char line_buf[10]; \
    itoa(line_buf, 'd', __LINE__); \
    terminal_writestring(line_buf); \
    terminal_writestring("\n"); \
    state = __LINE__; \
    return x; \
    case __LINE__:; \
} while (0)

#define crFinish }

/* Hardware text mode color constants. */
enum vga_color
{
  COLOR_BLACK = 0,
  COLOR_BLUE = 1,
  COLOR_GREEN = 2,
  COLOR_CYAN = 3,
  COLOR_RED = 4,
  COLOR_MAGENTA = 5,
  COLOR_BROWN = 6,
  COLOR_LIGHT_GREY = 7,
  COLOR_DARK_GREY = 8,
  COLOR_LIGHT_BLUE = 9,
  COLOR_LIGHT_GREEN = 10,
  COLOR_LIGHT_CYAN = 11,
  COLOR_LIGHT_RED = 12,
  COLOR_LIGHT_MAGENTA = 13,
  COLOR_LIGHT_BROWN = 14,
  COLOR_WHITE = 15,
};
 
uint8_t make_color(enum vga_color fg, enum vga_color bg)
{
  return fg | bg << 4;
}
 
uint16_t make_vgaentry(char c, uint8_t color)
{
  uint16_t c16 = c;
  uint16_t color16 = color;
  return c16 | color16 << 8;
}
 
size_t strlen(const char* str)
{
  size_t ret = 0;
  while ( str[ret] != 0 )
    ret++;
  return ret;
}
 
static const size_t VGA_WIDTH = 80;
static const size_t VGA_HEIGHT = 24;
 
size_t terminal_row;
size_t terminal_column;
uint8_t terminal_color;
uint16_t* terminal_buffer;

/* Terminal Functions */
void terminal_initialize()
{
  terminal_row = 0;
  terminal_column = 0;
  terminal_color = make_color(COLOR_LIGHT_GREY, COLOR_BLACK);
  terminal_buffer = (uint16_t*) 0xB8000;
  for ( size_t y = 0; y < VGA_HEIGHT; y++ )
    {
      for ( size_t x = 0; x < VGA_WIDTH; x++ )
	{
	  const size_t index = y * VGA_WIDTH + x;
	  terminal_buffer[index] = make_vgaentry(' ', terminal_color);
	}
    }
}
 
void terminal_setcolor(uint8_t color)
{
  terminal_color = color;
}
 
void terminal_putentryat(char c, uint8_t color, size_t x, size_t y)
{
  const size_t index = y * VGA_WIDTH + x;
  terminal_buffer[index] = make_vgaentry(c, color);
}
 
void terminal_putchar(char c)
{
  if (c == '\n') {
    terminal_column = 0;  // Move to the beginning of the next line
    if (++terminal_row == VGA_HEIGHT) {
        terminal_row = 0;  // Wrap around if we reach the bottom of the screen
    }
  } else {
      terminal_putentryat(c, terminal_color, terminal_column, terminal_row);
      if (++terminal_column == VGA_WIDTH) {
          terminal_column = 0;
          if (++terminal_row == VGA_HEIGHT) {
              terminal_row = 0;
          }
      }
  }
}
 
void terminal_writestring(const char* data)
{
  size_t datalen = strlen(data);
  for ( size_t i = 0; i < datalen; i++ )
    terminal_putchar(data[i]);
}
 

/* Convert the integer D to a string and save the string in BUF. If
   BASE is equal to 'd', interpret that D is decimal, and if BASE is
   equal to 'x', interpret that D is hexadecimal. */
void itoa (char *buf, int base, int d)
{
  char *p = buf;
  char *p1, *p2;
  unsigned long ud = d;
  int divisor = 10;
     
  /* If %d is specified and D is minus, put `-' in the head. */
  if (base == 'd' && d < 0)
    {
      *p++ = '-';
      buf++;
      ud = -d;
    }
  else if (base == 'x')
    divisor = 16;
     
  /* Divide UD by DIVISOR until UD == 0. */
  do
    {
      int remainder = ud % divisor;
     
      *p++ = (remainder < 10) ? remainder + '0' : remainder + 'a' - 10;
    }
  while (ud /= divisor);
     
  /* Terminate BUF. */
  *p = 0;
     
  /* Reverse BUF. */
  p1 = buf;
  p2 = p - 1;
  while (p1 < p2)
    {
      char tmp = *p1;
      *p1 = *p2;
      *p2 = tmp;
      p1++;
      p2--;
    }
}


/*--------------------- FIFOS 1 ---------------------*/

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
};  // attribute(packed) makes sure that there is no padding that is added.

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
#define GDT_SIZE 3

struct GDTDescriptor gdt[GDT_SIZE];

void setup_gdt() {
    // Null descriptor (required by the CPU)
    set_gdt_entry(&gdt[0], 0, 0, 0, 0);

    // Kernel Mode Code Segment
    set_gdt_entry(&gdt[1], 0x00000000, 0xFFFFF, 0x9A, 0x0C);

    // Kernel Mode Data Segment
    set_gdt_entry(&gdt[2], 0x00000000, 0xFFFFF, 0x92, 0x0C);

    /* 

      This section if for the user level.
      If we get to the Bonus, then this will be needed
    */
    // // User Mode Code Segment
    // set_gdt_entry(&gdt[3], 0x00000000, 0xFFFFF, 0xFA, 0x0C);

    // // User Mode Data Segment
    // set_gdt_entry(&gdt[4], 0x00000000, 0xFFFFF, 0xF2, 0x0C);
    
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
struct __attribute__((packed)) GDTR {
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

/*--------------------------------THREAD STUFF BEGIN--------------------------------*/
#define NUMOFTHREADS 5

void debug_halt() {
    terminal_writestring("Halting");
    while (1);
}
void schedule();

typedef enum {
    IDLE,
    READY,
    UNFINISHED,
    BUSY
} ThreadStatus;

typedef struct TCBlock {
    int tid;              // Thread ID
    uint32_t *stack_ptr;  // esp
    uint32_t *instr_ptr;  // eip
    ThreadStatus busy_flag; 
    uint32_t eax, ebx, ecx, edx, esi, edi, ebp; //(pushal/popal)
    uint32_t eflags;

} TCB;

TCB threads[NUMOFTHREADS];
int current_thread_id = 0; //global id tracker

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//1 will make it pring comment for grading purposes!
int print_flag = 0; 

typedef struct readyQ{
  TCB *buffer[NUMOFTHREADS];
  int head;
  int tail;
  int size;
} readyQ;

readyQ ready_queue; 

//exit func, gets put at the end of each thread
void exit() {
    threads[current_thread_id].busy_flag = IDLE; 
    threads[current_thread_id].instr_ptr = NULL;
    threads[current_thread_id].stack_ptr = NULL;

    if(print_flag){
    terminal_writestring("Thread exiting: ");
    char tid_buf[10];
    itoa(tid_buf, 'd', current_thread_id);
    terminal_writestring(tid_buf);
    terminal_writestring("\n");
    }

    schedule(); // exit back to scheduler

    terminal_writestring("Halt, no more threads\n");
    debug_halt();
}

void init_queue(readyQ *rq) {
  rq->head = 0;
  rq->tail = 0;
  rq->size = 0;
}

//2d array for allocating memory for stack, like what ta mention as the easiest way
//each thread get 4kb of memory
__attribute__((aligned(16))) uint8_t thread_stack_memory[NUMOFTHREADS][4096];

void setup_thread_gdt_entry(int thread_id) {
  uint32_t base = (uint32_t) &thread_stack_memory[thread_id];

  //Offset by 3 cuz they used for null, kernel code, and kernel data segments
  //base is starting spot, and 4095 is the size of stack, which I have set
  //0x92 is to access data seg, 0x0C is assosiate with the data seg on 
  //https://wiki.osdev.org/GDT_Tutorial
  set_gdt_entry(&gdt[thread_id + 3], base, 4095, 0x92, 0x0C);
}

void run_thread(int thread_id) {
  if(print_flag){
  terminal_writestring("Switching Thread: ");
  char tid_buf[10];
  itoa(tid_buf, 'd', thread_id);
  terminal_writestring(tid_buf);
  terminal_writestring("\n");
  }

  __asm__ volatile(
      "mov %0, %%esp\n" // Update stack pointer
      "jmp *%1\n" // Jump to instruction, ret baiscally
      :
      : "r"(threads[thread_id].stack_ptr), 
        "r"(threads[thread_id].instr_ptr)
  );
}

void save_thread(int thread_id) {
  if(print_flag){
  terminal_writestring("Saving thread : ");
  char tid_buf[10];
  itoa(tid_buf, 'd', current_thread_id);
  terminal_writestring(tid_buf);
  terminal_writestring("\n");
  }


  __asm__ volatile(
      "call 1f\n" // addres of *1, save current state of eip
      "1: pop %0\n"
      : "=m"(threads[current_thread_id].instr_ptr)
      :
      : "memory"
  );
  if(print_flag){
  terminal_writestring("Saved state of EIP: ");
  char eip_buf[10];
  itoa(eip_buf, 'x', (uint32_t)threads[current_thread_id].instr_ptr);
  terminal_writestring(eip_buf);
  terminal_writestring("\n");
  }

  __asm__ volatile(
      "mov %%esp, %0\n" // save current state of esp
      : "=m"(threads[current_thread_id].stack_ptr)
      :
      : "memory"
  );
  if(print_flag){
  terminal_writestring("Saved state of ESP: ");
  char esp_buf[10];
  itoa(esp_buf, 'x', (uint32_t)threads[current_thread_id].stack_ptr);
  terminal_writestring(esp_buf);
  terminal_writestring("\n");
  }

  // Manually pusha
  __asm__ volatile("mov %%eax, %0\n" : "=m"(threads[current_thread_id].eax) :: "memory");
  __asm__ volatile("mov %%ebx, %0\n" : "=m"(threads[current_thread_id].ebx) :: "memory");
  __asm__ volatile("mov %%ecx, %0\n" : "=m"(threads[current_thread_id].ecx) :: "memory");
  __asm__ volatile("mov %%edx, %0\n" : "=m"(threads[current_thread_id].edx) :: "memory");
  __asm__ volatile("mov %%esi, %0\n" : "=m"(threads[current_thread_id].esi) :: "memory");
  __asm__ volatile("mov %%edi, %0\n" : "=m"(threads[current_thread_id].edi) :: "memory");
  __asm__ volatile("mov %%ebp, %0\n" : "=m"(threads[current_thread_id].ebp) :: "memory");

  __asm__ volatile(
      "pushf\n" //save current eflags
      "pop %0\n"
      : "=m"(threads[current_thread_id].eflags)
      :
      : "memory"
  );
// now current thread state should be saved
}

void restore_thread(int thread_id) {
  if(print_flag){
  terminal_writestring("restoring: ");
  char tid_buf[10];
  itoa(tid_buf, 'd', current_thread_id);
  terminal_writestring(tid_buf);
  }
  __asm__ volatile(
      "mov %0, %%esp\n"
      :
      : "m"(threads[current_thread_id].stack_ptr)
      : "memory"
  );
  if(print_flag){
  char esp_buf[10];
  terminal_writestring("restored ESP: ");
  itoa(esp_buf, 'x', (uint32_t)threads[current_thread_id].stack_ptr);
  terminal_writestring(esp_buf);
  terminal_writestring("\n");
  }
    
  // Manually popa
  __asm__ volatile("mov %0, %%eax\n" : : "m"(threads[current_thread_id].eax) : "memory");
  __asm__ volatile("mov %0, %%ebx\n" : : "m"(threads[current_thread_id].ebx) : "memory");
  __asm__ volatile("mov %0, %%ecx\n" : : "m"(threads[current_thread_id].ecx) : "memory");
  __asm__ volatile("mov %0, %%edx\n" : : "m"(threads[current_thread_id].edx) : "memory");
  __asm__ volatile("mov %0, %%esi\n" : : "m"(threads[current_thread_id].esi) : "memory");
  __asm__ volatile("mov %0, %%edi\n" : : "m"(threads[current_thread_id].edi) : "memory");
  __asm__ volatile("mov %0, %%ebp\n" : : "m"(threads[current_thread_id].ebp) : "memory");

  __asm__ volatile(
      "push %0\n" 
      "popf\n" //restore the eflags
      :
      : "m"(threads[thread_id].eflags)
      : "memory"
  );

  if(print_flag){
  terminal_writestring("restored eip : ");
  char instr_buf[10];
  itoa(instr_buf, 'x', (uint32_t)threads[current_thread_id].instr_ptr);
  terminal_writestring(instr_buf);
  }
  //debug_halt();
  __asm__ volatile(
      "jmp *%0\n" // ret or go to where instr was saved
      :
      : "r"(threads[current_thread_id].instr_ptr)
      : "memory"
  );
}
void enque(readyQ *rq, TCB *tcb) {
  if (rq->size < NUMOFTHREADS) {
    rq->buffer[rq->tail] = tcb;
    rq->tail = (rq->tail + 1) % NUMOFTHREADS;
    rq->size++;
  } else {
    terminal_writestring("debug enq");
    debug_halt();
  }
}

TCB *deque(readyQ *rq) {
  if (rq->size > 0) {
    TCB *tcb = rq->buffer[rq->head];
    rq->head = (rq->head + 1) % NUMOFTHREADS;
    rq->size--;
    return tcb;
  }
  terminal_writestring("deque null\n");
  return NULL;
}

void init_threads() {
  for (int i = 0; i < 5; i++) {
    threads[i].tid = i;
    threads[i].stack_ptr = NULL;
    threads[i].instr_ptr = NULL;
    threads[i].busy_flag = IDLE;
    threads[i].eflags = 0x0;
  }
}

int threadCreate(void *func) {
  for (int i = 0; i < NUMOFTHREADS; i++) {
    if (threads[i].busy_flag == IDLE) {
      threads[i].instr_ptr = func;    
      threads[i].busy_flag = READY;

      //new gdt use
      uint32_t *stack = (uint32_t *)&thread_stack_memory[i][4096];
      threads[i].stack_ptr = stack;
      setup_thread_gdt_entry(i);  // setup gdt entry for thread
      enque(&ready_queue, &threads[i]); 
      return threads[i].tid;
    }
  }
  return -1;
}

void yield() {
  if(print_flag){
  terminal_writestring("Thread that is yielding: ");
  char tid_buf[10];
  itoa(tid_buf, 'd', current_thread_id);
  terminal_writestring(tid_buf);
  terminal_writestring("\n");
  }
  threads[current_thread_id].busy_flag = UNFINISHED;
  enque(&ready_queue, &threads[current_thread_id]); 

  schedule();
}

void schedule(void) {
  if (ready_queue.size == 0) {
      terminal_writestring("No threads to schedule\n");
      debug_halt(); 
      return;
  }

  // Find next thread
  TCB *next_thread = deque(&ready_queue);

if (next_thread == NULL) {
    terminal_writestring("Q is NULL\n");
    debug_halt(); 
}
    //stores current thread state its unfinished
    if(threads[current_thread_id].busy_flag == UNFINISHED){
    save_thread(current_thread_id);
    }
    //debug_halt();

    //switch stacks and restore
    current_thread_id = next_thread->tid;
    //case where the thread is fresh / instructions have not been ran yet
    if (threads[current_thread_id].busy_flag == READY) { 
      // terminal_writestring("running fresh thread\n");
      run_thread(current_thread_id);
      return;

    //case where its unfinished so it has a saved state to be restored
    } else if(threads[current_thread_id].busy_flag == UNFINISHED) {
      restore_thread(current_thread_id);
      return;
    }
    terminal_writestring("got to the end of sch O_O\n");
    debug_halt(); 
    return;
}

/////////testing functions//////////

void func1() {

    terminal_writestring("<0>Checkpint 1\n");
    yield();

    terminal_writestring("<0> checkpoint: 2 \n");
    yield(); 

    terminal_writestring("<0> checkpoint: 3 \n");
    yield(); 

    terminal_writestring("Done <0>!\n");
    exit();
}

void func2() {
    terminal_writestring("<1>Checkpint 1\n");
    yield();

    terminal_writestring("<1> checkpoint: 2 \n");
    yield(); 

    terminal_writestring("<1> checkpoint: 3 \n");
    yield(); 

    terminal_writestring("Done <1>!\n");
    exit();
}

void func3() {
    terminal_writestring("<2> Checkpint: 1\n");
    yield();

    terminal_writestring("<2> checkpoint: 2 \n");
    yield(); 

    terminal_writestring("<2> checkpoint: 3 \n");
    yield(); 

    terminal_writestring("Done <2>!\n");
    exit();
}









/* DISCOS IMPLEMENTATION *///////////////////////////////////////

#define MEMORY_SIZE 0x200000
#define BLOCK_SIZE 256
#define NODE_SIZE 64
#define MAX_FILE_OBJECTS (3 * 16) // 3 threads and 16 open files each

// #define BIT_0 0b00000001
// #define BIT_1 0b00000010
// #define BIT_2 0b00000100
// #define BIT_3 0b00001000
// #define BIT_4 0b00010000
// #define BIT_5 0b00100000
// #define BIT_6 0b01000000
// #define BIT_7 0b10000000


/* 
  Superblock specific implementation.

  # of free blocks = RAM DISK SIZE (2 MB) / block size (256 byte) = 8192 blocks 
  # of free index node = 256 * 4

*/
typedef struct {
  uint32_t free_blocks;     // # of free blocks
  uint32_t free_index_node; // # of free index nodes
} super_block_t __attribute__((aligned(256)));

/* File type enum */
typedef enum {
  invalid = 0,
  reg,
  dir
} file_type_t;

/* Address status enum for bitmap */
typedef enum {
  allocated = 0,
  free
} address_status_t;

typedef struct {
  uint32_t direct_pointer[8]; // Direct pointers
  uint32_t single_indir; // pointer to a block with 64 pointers
  uint32_t double_indir; // pointer to a block wth 64 single indrect blocks
} location_t;

typedef struct {
  uint32_t single_indir_pointer[64];
} single_indir_t __attribute__((aligned(256)));

typedef struct {
  uint32_t double_indir_pointer[64];
} double_indir_t __attribute__((aligned(256)));

/* Index node */
typedef struct {
  file_type_t type;           // "reg" for regular or "dir" for directory (4 bytes)
  uint32_t size;              // size of the file (4 bytes)
  location_t location;        // identifies the block that store the file contents (40 bytes)
} index_node_t;

typedef struct {
  index_node_t node_array[256][4];
} index_node_array_t;

/* Block bitmap */
typedef struct {
  uint8_t bitmap[1024];
} block_bitmap_t __attribute__((aligned(1024)));

/* Data block */
// This will be the structure for one sub-directory
typedef struct __attribute__((__packed__)){
  char filename[14];
  uint16_t index_node_number;
} directory_entries_t;

typedef struct {
  uint8_t data[256];
} regular_file_t;

typedef struct __attribute__((aligned(256))){
  union {
    directory_entries_t entries[16];
    regular_file_t reg_file;
  } fileType;
} data_block_t;

/* data block array */
typedef struct {
  // Array of data block. 8192 - 1 (superblock) - 256 (index node array) - 4 (bitmap) = 7931 db
  data_block_t db_array[7931];
} data_block_array_t;

/* RAM memory */
typedef struct {
  uint32_t base_address;
  uint32_t size;
  super_block_t *super_block;
  index_node_array_t *index_node_array;
  block_bitmap_t *bitmap;
  data_block_array_t *db_array;
} ram_disk_t;

/* This is used for bitmap */
typedef enum {
  INODE,      // Index node block
  DATA_BLOCK  // Data block
} block_type_t;

typedef struct {
  uint32_t file_pos;
  index_node_t *inode;
}file_object_t;

typedef volatile uint32_t spinlock_t;

typedef struct {
  file_object_t *open_files[16];
  spinlock_t lock;
}fd_table_t;

fd_table_t fd_table[3];
file_object_t file_objects[MAX_FILE_OBJECTS];
/* Global variable */
ram_disk_t ram_disk;

// spin lock for sync and fd table stuff

void spinlock_init(spinlock_t *lock) {
  *lock = 0;
}

void spinlock_lock(spinlock_t *lock) {
  while (__sync_lock_test_and_set(lock, 1)) {
    while (*lock);
  }
}

void spinlock_unlock(spinlock_t *lock) {
  __sync_lock_release(lock);
}

file_object_t *free_file_obj() {
  for (int i = 0; i< MAX_FILE_OBJECTS; i++) {
    if (file_objects[i].inode == NULL) {
      return &file_objects[i];
    }
  }
  return NULL;
}


/* INITIALIZATION FUNCTIONS */  

/* find the memory address where there is contiguous 2 MB */
uint32_t find_memory ( multiboot *pmb ) {
  memory_map_t *mmap;

  for (mmap = (memory_map_t *) pmb->mmap_addr; (unsigned long) mmap < pmb->mmap_addr + pmb->mmap_length; mmap = (memory_map_t *) ((unsigned long) mmap + mmap->size + 4 /*sizeof (mmap->size)*/)) { 
    if ((mmap->type == 1) && (mmap->length_low >= 0x200000)) {
      return mmap->base_addr_low;
    }
  }

  // Reach this if we fail to find memory
  return 1;
}

/* Initialize super block*/
void init_superblock (super_block_t *super_block) {
  // char buffer[10];
  super_block->free_blocks = (0x200000 / 256) - 1 - 4;
  super_block->free_index_node = 256 * 4;

  // itoa(buffer, 'd', super_block->free_blocks);
  // terminal_writestring("# of free blocks: ");
  // terminal_writestring(buffer);
  // terminal_writestring("\n");

  // itoa(buffer, 'd', super_block->free_index_node);
  // terminal_writestring("# of free index: ");
  // terminal_writestring(buffer);
  // terminal_writestring("\n");
}

/* every bitmap should be free to use*/
void init_bitmap (block_bitmap_t *bitmap) {
  for (int i = 0; i < 1024; i++) {
    bitmap->bitmap[i] = 0b11111111;
  }
  // superblock
  bitmap->bitmap[0] = 0b11111110;

  // bit map are always occupied
  // Mark bitmap blocks (blocks 257-260) as allocated
  bitmap->bitmap[32] = 0b11100001;
}

/* Initialize root now! */
void init_root (index_node_array_t *index_node_array, block_bitmap_t *bitmap, data_block_array_t *db_array) {
  /* data block */
  strncpy(db_array->db_array[0].fileType.entries->filename, "");
  db_array->db_array[0].fileType.entries->index_node_number = 0;

  /* set the 0-th index node to root */
  index_node_array->node_array[0][0].type = dir;
  index_node_array->node_array[0][0].size = 0;
  index_node_array->node_array[0][0].location.direct_pointer[0] = 0;

  /* manual bitmap assignment */
  bitmap->bitmap[32] &= ~(1 << 5);

  // This is the root dirctory
  ram_disk.super_block->free_blocks -= 1;
  ram_disk.super_block->free_index_node -= 1;


  // we dont label 0 for the inode bitmap because the the block still can take 3 more inodes.
}

void init_memory( multiboot* pmb ) {
  /* Find memory here */
  ram_disk.base_address = find_memory(pmb);
  ram_disk.size = 0x200000;

  ram_disk.super_block = (super_block_t *) ram_disk.base_address;
  ram_disk.index_node_array = (index_node_array_t *) (ram_disk.base_address + sizeof(super_block_t));
  ram_disk.bitmap = (block_bitmap_t *) (ram_disk.base_address + sizeof(super_block_t) + sizeof(index_node_array_t));
  ram_disk.db_array = (data_block_array_t *) (ram_disk.base_address + sizeof(super_block_t) + sizeof(index_node_array_t) + sizeof(block_bitmap_t));
  
  init_superblock(ram_disk.super_block);
  init_bitmap(ram_disk.bitmap);

  // init stuff for fd table
  for (int i = 0; i < 3; i++) {
    spinlock_init(&fd_table[i].lock);
    for (int j = 0; j < 16; j++) {
      fd_table[i].open_files[j] = NULL;    
    }
  }

  for (int i = 0; i < MAX_FILE_OBJECTS; i++) {
    file_objects[i].file_pos = 0;
    file_objects[i].inode = NULL; 
  }
  

  /* set up the root directory */
  init_root(ram_disk.index_node_array, ram_disk.bitmap, ram_disk.db_array);


  // terminal_writestring(ram_disk.db_array->db_array[0].fileType.dir_entry.filename);
}

/* 
  Find free block(either returns inode or the data block).
  Note that it will mark the block as allocated at the bitmap index
 */
int is_allocated_bitmap(block_bitmap_t *bitmap, block_type_t type) {
  int start_index, end_index;
  // Determine the range of the bitmap to search
  if (type == INODE) {
      start_index = 0;      // INODE blocks start at the beginning of the bitmap
      end_index = 33;       // End of the INODE blocks (at the 0th bit of the 32nd bitmap index)
  } else if (type == DATA_BLOCK) {
      start_index = 32;     // DATA_BLOCK blocks start at byte 32 (block 256 onwards)
      end_index = 1024;     // End of the bitmap
  } else {
      return -1; // Invalid block type
  }

  // Iterate through the specified range of the bitmap
  for (int i = start_index; i < end_index; i++) {
      if (bitmap->bitmap[i] != 0) { // If this byte has at least one free block
        if (type == DATA_BLOCK && i == 32) {
            // Handle bitmap[32] specifically for data block allocation
            if (bitmap->bitmap[32] == 1) {
                continue; // Skip entirely if only the first bit is allocated
            }
            for (int bit = 6; bit < 8; bit++) { // Bits 6 and 7 are for data blocks
                if (bitmap->bitmap[32] & (1 << bit)) {
                    bitmap->bitmap[32] &= ~(1 << bit); // Mark the block as allocated
                    int block_number = i * 8 + bit - 261; // Adjust block number
                    return block_number;
                }
            }
            continue; // Skip to the next byte if no valid data blocks
        }
        
        for (int bit = 0; bit < 8; bit++) {
          if (bitmap->bitmap[i] & (1 << bit)) {
            int block_number = i * 8 + bit;

            if (type == INODE) {
              // Check all 4 inode slots in this block
              int all_used = 1;  // Flag to track if all slots are used
              for (int col = 0; col < 4; col++) {
                  if (ram_disk.index_node_array->node_array[block_number][col].type == invalid) {
                      // Allocate this inode slot
                      ram_disk.index_node_array->node_array[block_number][col].type = reg;
                      ram_disk.super_block->free_index_node--;

                      // Check if this block is fully used
                      for (int k = 0; k < 4; k++) {
                        if (ram_disk.index_node_array->node_array[block_number][k].type == invalid) {
                          all_used = 0;  // Found a free slot
                          break;
                        }
                      }

                      // Clear the bitmap bit only if all slots are used
                      if (all_used) {
                        bitmap->bitmap[i] &= ~(1 << bit);
                        ram_disk.super_block->free_blocks--;
                      }

                      return block_number * 4 + col;  // Return unique inode index
                  }
              }
           }

            if (type == DATA_BLOCK) {
              // terminal_writestring("block number in data block: ");
              // itoa(buffer, 'd', block_number);
              // terminal_writestring(buffer);
              // terminal_writestring("\n");
              bitmap->bitmap[i] &= ~(1 << bit); // Mark block as used
              ram_disk.super_block->free_blocks--;
              return block_number - 261; // Adjust for data block array index
            }
          }
        }
      }
  }

  return -1; // No free blocks
}



/* file operations!!!! */
void strncpy(char *dest, const char *src) {
  int i = 0;

  // Copy characters from src to dest up to 14 bytes
  while (i < 14 && src[i] != '\0') {
    dest[i] = src[i];
    i++;
  }

  // Fill the rest with null terminators to ensure proper padding
  while (i < 14) {
    dest[i] = '\0';
    i++;
  }
}


int strcmp(char *a, char *b) {
  for (int i = 0; i < 14; i++) { // Compare up to 14 characters
    if (a[i] != b[i]) return 0; // Mismatch
    if (a[i] == '\0') return 1; // Both strings end
  }
  return 1;  // Strings are equal up to 14 bytes
}



/* search directory */
int search_dir(char *filename, int iNodeNumber) {
  
  int iNodeRow = iNodeNumber / 4;
  int iNodeCol = iNodeNumber % 4;
  int currDBIndex;
  int nextINodeNum;
  int checkInode;
  int checkRow;
  int checkCol;
  
  for (int j = 0; j < 8; j++) {
    currDBIndex = ram_disk.index_node_array->node_array[iNodeRow][iNodeCol].location.direct_pointer[j];
    // Traverse the directory entries
    for (int i = 0; i < 16; i++) {
      // terminal_writestring("traverse huh ?\n");
      if (strcmp(ram_disk.db_array->db_array[currDBIndex].fileType.entries[i].filename, filename)) {
        checkInode = ram_disk.db_array->db_array[currDBIndex].fileType.entries[i].index_node_number;
        checkRow = checkInode / 4;
        checkCol = checkInode % 4;
        // terminal_writestring("traverse huh ??\n");
        if (ram_disk.index_node_array->node_array[checkRow][checkCol].type == dir) {
          // Found the file in the directory and we need to update the 
          nextINodeNum = checkInode;
          return nextINodeNum; // File found
        }
      }
    }
  }
  // We need to handle the single pointer
  
  // Handle double pointer

  return -1; // File not found
}



/* Traverse the path */
int traverse_path(char *pathname) {

  // We start searching thru the file tree from the root directory
  // the root directory starts from the 0th index of the inode and the data block.
  char token[10][14];
  int currToken = 0;
  int iNodeIndex = 0; // Start from 0th index (root directory)
  // terminal_writestring("traverse huh 1\n");
  // Get the length and the token
  int tokenLength = tokenize(pathname, token);
  // terminal_writestring("traverse huh 2\n");
  // char buffer[10];
  // itoa(buffer, 'd', tokenLength);
  // terminal_writestring("token length: ");
  // terminal_writestring(buffer);
  // terminal_writestring("\n");
  if (tokenLength == 1) {
    return iNodeIndex;
  }

  // terminal_writestring("traverse huh 3\n");
  while (currToken < tokenLength - 1) {
    // Case when the file is not found
    iNodeIndex = search_dir(token[currToken], iNodeIndex);
    if(iNodeIndex == -1){
      return -1;
    } else {
      currToken++;
      if (currToken == tokenLength - 1) {
        return iNodeIndex;
      }
    }
  }
  return -1;
}


int tokenize(char *pathname, char token[10][14]) {
  // pathname = "/dir1/dir2/file1";
  /* Each array index will have string of the file */
    
    int pathIndex = 0;      // Current index in pathname
    int tokenIndex = 0;     // Current index in the current token
    int currentToken = 0;   // Current token index

    // Check for an empty path
    if (pathname[0] == '\0') {
        return -1;  // Do nothing if the string is empty
    } else if (pathname[0] == '/') {
        pathIndex++;  
    }

    // Tokenize the pathname
    while (pathname[pathIndex] != '\0') {
        if (pathname[pathIndex] == '/') {
            // Null-terminate the current token
            token[currentToken][tokenIndex] = '\0';
            currentToken++;  // Move to the next token
            tokenIndex = 0;  // Reset index for the new token

            if (currentToken >= 10) {
                break;  // Prevent overflow of the token array
            }
        } else {
            // Add character to the current token
              token[currentToken][tokenIndex] = pathname[pathIndex];
              tokenIndex++;
        }
        pathIndex++;
    }

    // Null-terminate the last token (if any)
    if (tokenIndex > 0) {
        token[currentToken][tokenIndex] = '\0';
        currentToken++;
    }
    return currentToken;
}

/* 
  Create a regular file at the specified path.
  Returns 0 on success, -1 on failure.
 */
int rd_creat(char *pathname) {
  int index_block_number, data_block_number;
  // TODO: traverse the path and make sure the path is valid.
  char token[10][14];
  int parentDirectoryNode;
  int indexOfFilename;

  // int row, col;

  indexOfFilename = tokenize(pathname, token);
  // terminal_writestring("huh 1\n");
  parentDirectoryNode = traverse_path(pathname);
  // terminal_writestring("huh 2\n");
  if (parentDirectoryNode == -1) {
    return -1; // Invalid path
  }
  
  if (ram_disk.super_block->free_index_node == 0) {
    return -1; // No free blocks
  }
  // terminal_writestring("huh 3\n");
  // We gonna get the index and data block number that is free. (and yes the function will mark it as allocated)
  index_block_number = is_allocated_bitmap(ram_disk.bitmap, INODE);
  // terminal_writestring("huh 4\n");
  data_block_number = is_allocated_bitmap(ram_disk.bitmap, DATA_BLOCK);
  // terminal_writestring("huh 5\n");

  // row = index_block_number / 4;
  // col = index_block_number % 4;

  // ram_disk.index_node_array->node_array[row][col].type = reg;
  // ram_disk.index_node_array->node_array[row][col].size = 0;
  // for (int j = 0; j < 8; j++) {
  //   if (ram_disk.index_node_array->node_array[row][col].location.direct_pointer[j] < 0) {
  //     ram_disk.index_node_array->node_array[row][col].location.direct_pointer[j] = data_block_number;
  //   }
  // }


  // Set the inode
  // for (int i = 0; i < 4; i++) {
  //   if (ram_disk.index_node_array->node_array[index_block_number][i].type == invalid) {
  //     ram_disk.index_node_array->node_array[index_block_number][i].type = reg;
  //     ram_disk.index_node_array->node_array[index_block_number][i].size = 0;
  //     for (int j = 0; j < 8; j++) {
  //       if (ram_disk.index_node_array->node_array[index_block_number][i].location.direct_pointer[j] < 0) {
  //         ram_disk.index_node_array->node_array[index_block_number][i].location.direct_pointer[j] = data_block_number;
  //       }
  //     }
  //     break;
  //   }
  // }
  // terminal_writestring("huh 6\n");

  // Set the data block
  // ram_disk.db_array->db_array[data_block_number].fileType.reg_file.data[0] = '\0';

  // terminal_writestring("huh 7\n");
  // TO DO: We gotta traverse the path and set the directory entry in the parent 
  // directory to point to the new file.
  // for (int i = 0; i < 8; i++) {
  //   if (ram_disk.db_array->db_array[parentDirectoryNode].fileType.entries[i].index_node_number < 0) {
  //     strncpy(ram_disk.db_array->db_array[parentDirectoryNode].fileType.entries[i].filename, token[indexOfFilename]);
  //     ram_disk.db_array->db_array[parentDirectoryNode].fileType.entries[i].index_node_number = index_block_number;
  //   }
  // }
  // terminal_writestring("huh 8\n");

  return 0;
}

int rd_open(int thread_id, char *pathname) {
  
  //find inode at the end of path and find pos
  int iNode_num = traverse_path(pathname); 
  int row = iNode_num /4;
  int col = iNode_num % 4;

  //search for a slot in the fd table if any
  for (int i = 0; i < 16; i++) {
    if (fd_table[thread_id].open_files[i] == NULL) {
      file_object_t *file = free_file_obj();
      if (file == NULL) {
        return -1;
      }

      file->file_pos = 0;
      file->inode = &ram_disk.index_node_array->node_array[row][col];
      fd_table->open_files[i] = file;
      return i; // Return file descriptor
    }
  }
  return -1;
}
#define TEST_RD_CREATE


#define MAX_FILES 1023        // Maximum files supported per directory
#define PATH_PREFIX ""   // Path prefix for files
#ifdef TEST_RD_CREATE
void test_rd_create() {
    char pathname[80];
    int retval;
    int i;

    terminal_writestring("/////////////////TEST 1 MAX FILES /////////////////\n");

    // string stuff to pass make files with, files0, files1 ...
    for (i = 0; i < MAX_FILES + 1; i++) { // Go beyond the limit to test failure
        int pos = 0; // Position in the pathname buffer
        for (int j = 0; PATH_PREFIX[j] != '\0'; j++) {
            pathname[pos++] = PATH_PREFIX[j]; // Copy PATH_PREFIX
        }
        pathname[pos++] = '/'; // Add '/' after PATH_PREFIX

        char file_num[10]; // Buffer for the file number
        itoa(file_num, 'd', i);

        char file_prefix[] = "file";
        for (int j = 0; file_prefix[j] != '\0'; j++) {
            pathname[pos++] = file_prefix[j]; // Append "file"
        }

        for (int j = 0; file_num[j] != '\0'; j++) {
            pathname[pos++] = file_num[j]; // Append file number
        }

        pathname[pos] = '\0'; // Null-terminate the string

        // Try creating the file
        retval = rd_creat(pathname);

        if (retval < 0) {
            terminal_writestring("rd_creat: File creation error! File: ");
            terminal_writestring(pathname);
            terminal_writestring("\n");

            // Fail only if we haven't yet reached the max limit
            if (i != MAX_FILES) {
                terminal_writestring("TEST FAILED! File creation limit reached prematurely.\n");
                debug_halt();
            }
        }

        // Clear pathname for reuse
        for (int j = 0; j < 80; j++) pathname[j] = '\0';
    }

    terminal_writestring("All files created successfully.\n");

  

    // terminal_writestring("All files deleted successfully.\n");
    terminal_writestring("////// TEST: max file create only no delte //////\n");
}

#endif

void init( multiboot* pmb ) {
  /* Screen reset */
  terminal_initialize();

  // initialize memory (including root setup)
  terminal_writestring("Welcome to Kim OS!");

  init_memory(pmb);
  char buffer[10];

  terminal_writestring("superblock free blocks: ");
  itoa(buffer, 'd', ram_disk.super_block->free_blocks);
  terminal_writestring(buffer);
  terminal_writestring("\n");
  
  terminal_writestring("superblock free inodes: ");
  itoa(buffer, 'd', ram_disk.super_block->free_index_node);
  terminal_writestring(buffer);
  terminal_writestring("\n");

  #ifdef TEST_RD_CREATE
    test_rd_create();
  #endif
  // terminal_writestring("-------after--------\n");
  // terminal_writestring("8 bitmap for inode: [");
  // terminal_writestring(" ");
  // for (int i = 0; i < 32; i++) {
  //   itoa(buffer, 'd', ram_disk.bitmap->bitmap[i]);
  //   terminal_writestring(buffer);
  //   terminal_writestring(" ");
  // }
  // terminal_writestring("]"); 
  // terminal_writestring("\n"); 

  // terminal_writestring("8 bitmap for data block: [");
  //   terminal_writestring(" ");
  // for (int i = 32; i < 256; i++) {
  //   itoa(buffer, 'd', ram_disk.bitmap->bitmap[i]);
  //   terminal_writestring(buffer);
  //   terminal_writestring(" ");
  // }
  // terminal_writestring("]"); 
  // terminal_writestring("\n"); 


  terminal_writestring("superblock free blocks: ");
  itoa(buffer, 'd', ram_disk.super_block->free_blocks);
  terminal_writestring(buffer);
  terminal_writestring("\n");
  
  terminal_writestring("superblock free inodes: ");
  itoa(buffer, 'd', ram_disk.super_block->free_index_node);
  terminal_writestring(buffer);
  terminal_writestring("\n");

  




  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //SET PRINT FLAG TO 1 FOR MEMORY PRINTS
  //print_flag = 1; 
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  setup_gdt();
  load_gdt();
  // debug_gdtr();



  /* This is for thread, temp taken out*/
  // init_threads();
  // init_queue(&ready_queue);
  //debug_halt();

  // terminal_writestring("cooking threads\n");
  // threadCreate(func1);
  // threadCreate(func2);
  // threadCreate(func3);

  /*
    TCB *first_thread = deque(&ready_queue);
    if (first_thread != NULL) {
        current_thread_id = first_thread->tid;
        // terminal_writestring("Starting first thread: ");
        // char tid_buf[10];
        // itoa(tid_buf, 'd', current_thread_id);
        // terminal_writestring(tid_buf);
        // terminal_writestring("\n");
        threads[current_thread_id].busy_flag = BUSY;
        run_thread(first_thread->tid);
    }
  */

  // while (ready_queue.size > 0) {
  //   schedule();
  // }
  debug_halt();
}
