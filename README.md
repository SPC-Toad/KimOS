# Kim OS
- Welcome to my personal Operating System
## *About*
- Created 12/29/24!
- This is where I learn and apply my OS knowledge 
- This is a long term project [ `never-ending project :)` ] (Update: this is 32 bit os, I am building another one that is 64 bit OS)
- If you want to know what I have learned during OS journey, check out the `sys-lvl-notes` in my github repo and `notes` folder in the KIMOS.

### Functionalities
#### Completed
1. GRUB 2 setup (old setup was using GRUB legacy --v 0.97)
2. memory mapping (using multiboot)
3. Fragmentation (modulizing all the work into header files)



#### To do list
- Context switching (FIFO with yield) {under construction `refining phase`}
- `Full` file system support {under construction `support for the indirect pointers and proper bitmap counter`}
    - correct index node tracking (in bitmap)
    - Support single and double indirect pointers
- Setup IDT (Interrupt descripter table) {under construction}
1. Keyboard driver (requires IDT and IRQ 1 call)
2. Context switching (Round Robin(RR) by using PIT for time)
4. So much more!

<hr>

## DEMO: `Current version`
![image](https://github.com/user-attachments/assets/12b25020-6565-4e62-a0cc-6c38cb8561aa)



<br>

If you want to contribute to this OS, join me!


### Contributor 
- Sangyun Kim
- Long Luu

## I will probably change the name of the OS if we get dev and community going.
