#include "multiboot.h"
#include "types.h"
#define NULL ((void *)0)
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

    // Code segment descriptor
    set_gdt_entry(&gdt[1], 0x00000000, 0xFFFFF, 0x9A, 0x0C);

    // Data segment descriptor
    set_gdt_entry(&gdt[2], 0x00000000, 0xFFFFF, 0x92, 0x0C);
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

  __asm__ __volatile__("lgdt %0" : : "m"(gdtr));

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
    terminal_writestring("\nHalting\n");
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

  __asm__ __volatile__(
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


  __asm__ __volatile__(
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

  __asm__ __volatile__(
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
  __asm__ __volatile__("mov %%eax, %0\n" : "=m"(threads[current_thread_id].eax) :: "memory");
  __asm__ __volatile__("mov %%ebx, %0\n" : "=m"(threads[current_thread_id].ebx) :: "memory");
  __asm__ __volatile__("mov %%ecx, %0\n" : "=m"(threads[current_thread_id].ecx) :: "memory");
  __asm__ __volatile__("mov %%edx, %0\n" : "=m"(threads[current_thread_id].edx) :: "memory");
  __asm__ __volatile__("mov %%esi, %0\n" : "=m"(threads[current_thread_id].esi) :: "memory");
  __asm__ __volatile__("mov %%edi, %0\n" : "=m"(threads[current_thread_id].edi) :: "memory");
  __asm__ __volatile__("mov %%ebp, %0\n" : "=m"(threads[current_thread_id].ebp) :: "memory");

  __asm__ __volatile__(
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
  __asm__ __volatile__(
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
  __asm__ __volatile__("mov %0, %%eax\n" : : "m"(threads[current_thread_id].eax) : "memory");
  __asm__ __volatile__("mov %0, %%ebx\n" : : "m"(threads[current_thread_id].ebx) : "memory");
  __asm__ __volatile__("mov %0, %%ecx\n" : : "m"(threads[current_thread_id].ecx) : "memory");
  __asm__ __volatile__("mov %0, %%edx\n" : : "m"(threads[current_thread_id].edx) : "memory");
  __asm__ __volatile__("mov %0, %%esi\n" : : "m"(threads[current_thread_id].esi) : "memory");
  __asm__ __volatile__("mov %0, %%edi\n" : : "m"(threads[current_thread_id].edi) : "memory");
  __asm__ __volatile__("mov %0, %%ebp\n" : : "m"(threads[current_thread_id].ebp) : "memory");

  __asm__ __volatile__(
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
  __asm__ __volatile__(
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


void init( multiboot* pmb ) {
   memory_map_t *mmap;
   unsigned int memsz = 0;		/* Memory size in MB */
   static char memstr[10];

  for (mmap = (memory_map_t *) pmb->mmap_addr;
       (unsigned long) mmap < pmb->mmap_addr + pmb->mmap_length;
       mmap = (memory_map_t *) ((unsigned long) mmap
				+ mmap->size + 4 /*sizeof (mmap->size)*/)) { 
    if (mmap->type == 1)	/* Available RAM -- see 'info multiboot' */
      memsz += mmap->length_low;
  }

  /* Convert memsz to MBs */
  memsz = (memsz >> 20) + 1;	/* The + 1 accounts for rounding
				   errors to the nearest MB that are
				   in the machine, because some of the
				   memory is othrwise allocated to
				   multiboot data structures, the
				   kernel image, or is reserved (e.g.,
				   for the BIOS). This guarantees we
				   see the same memory output as
				   specified to QEMU.
				    */

  itoa(memstr, 'd', memsz);

  terminal_initialize();

  terminal_writestring("FIFOS: Welcome *** System memory is: ");
  terminal_writestring(memstr);
  terminal_writestring("MB");

  //testing 3 threads for assign 1//

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //SET PRINT FLAG TO 1 FOR MEMORY PRINTS
  //print_flag = 1; 
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  setup_gdt();
  load_gdt();
  debug_gdtr();
  init_threads();
  init_queue(&ready_queue);
  //debug_halt();

  terminal_writestring("cooking threads\n");
  threadCreate(func1);
  threadCreate(func2);
  threadCreate(func3);

  // TCB *first_thread = deque(&ready_queue);
  // if (first_thread != NULL) {
  //     current_thread_id = first_thread->tid;
  //     // terminal_writestring("Starting first thread: ");
  //     // char tid_buf[10];
  //     // itoa(tid_buf, 'd', current_thread_id);
  //     // terminal_writestring(tid_buf);
  //     // terminal_writestring("\n");
  //     threads[current_thread_id].busy_flag = BUSY;
  //     run_thread(first_thread->tid);
  // }

  while (ready_queue.size > 0) {
    schedule();
  }
  debug_halt();
}