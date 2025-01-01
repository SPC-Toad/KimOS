# How disk image works (grub 2)

1. Create disk image. 
    - We need to make sure we are making 60480 sectors.
    Cylinders = 60, Heads= 16, Sectors = 63
    Where C * H * S = 60480 with each sector being 512 bytes (total of 30965760 byte or 30.9MB).
```sh
    $dd if=/dev/zero of=kimOS.img bs=512 count=60480
```

2. Partition the disk
    - We will be using only one partition.
```sh
fdisk kimOS.img
# Inside fdisk:
# expert mode by typeing x
# n → p → 1 → Enter for defaults
# t → 83 (Linux)
# a → 1 (set bootable)
# w (write and exit)
```

3. Setup loop device
```sh
$sudo losetup -Pf kimOS.img
```

4. Format the partition
    - We will use ext2 for now
```sh
$sudo mkfs.ext2 /dev/loop{loop `#` at which the kimOS is at}p1
```

5. Mount the partition
    - This is where you modify the kimOS
```sh
$sudo mount /dev/loopXp1 ./mnt
```

6. Install GRUB 2 in the `mnt`
```sh
sudo grub-install --boot-directory=./mnt/boot --target=i386-pc /dev/loopX
```

7. Copy kernel
```sh
$sudo cp ./kimOS.elf ./mnt/boot
```

8. GRUB configuration
    - This is basically menu.lst file for the GRUB legacy
```sh
$sudo nano ./mnt/boot/grub/grub.cfg
```
```vim
set timeout=0
set default=0

menuentry "kimOS" {
    set root=(hd0,1)
    multiboot /boot/kernel.elf
    boot
}
```

9. Unmount and detach image file
```sh
$sudo umount ./mnt
$sudo losetup -d /dev/loopX
```

10. Run the qemu!
```sh
$make run
```

# I don't care about the specific, I want to edit the OS and just see the changes I make!

## Ok here is the Makefile.
```sh
# Erase the elf binary file (Recommended but can be skipped)
$make clean

# Create static library (*.a file) {it will automatically compile all the module c file --> obj file}
$make library

# Compile c file to elf binary file
$make kernel

# Mount the OS disk img to the loop device
$make mount

# Add the updated elf bin file to the OS disk img
$make update

# Unmount the OS disk img since we made changes
$make umount

# Let's run out OS disk
$make run
```