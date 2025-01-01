CC=gcc
AS=as
AR=ar
CFLAGS=-m32 -fno-stack-protector -fno-builtin -nostdinc
LDFLAGS=-m elf_i386
LIB=libkernel.a
MODULE_SRC=$(wildcard module/*/*.c)
MODULE_OBJS=$(MODULE_SRC:.c=.o)

run:
	qemu-system-i386 -hda OS.img
# This will put all the obj file into a static library
library: $(MODULE_OBJS)
	$(AR) rcs $(LIB) $(MODULE_OBJS) 
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
kernel:
	$(AS) --32 bootloader.S -o bootloader.o
	$(CC) $(CFLAGS) -c OS.c -o kimOS.o 
	ld $(LDFLAGS) -T kimOS.ld bootloader.o kimOS.o -L. -lkernel -o kimOS.elf
mount:
	sudo losetup -Pf OS.img && \
	LOOP_DEVICE=$$(losetup -j OS.img | awk -F':' '{print $$1}'); \
	echo "Loop device is: $${LOOP_DEVICE}"; \
	sudo mount $${LOOP_DEVICE}p1 ./mnt
update:
	sudo cp ./kimOS.elf ./mnt/boot
umount:
	sudo umount ./mnt && \
	LOOP_DEVICE=$$(losetup -j OS.img | awk -F':' '{print $$1}'); \
	echo "Loop device is: $${LOOP_DEVICE}"; \
	sudo losetup -d $${LOOP_DEVICE}
clean:
	rm -f $(MODULE_OBJS) $(LIB) kimOS.elf kimOS.o bootloader.o

