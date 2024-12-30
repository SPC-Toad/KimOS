run:
	qemu-system-i386 -hda kimOS.img
kernel:
	as --32 bootloader.S -o bootloader.o
	gcc -m32 -fno-stack-protector -fno-builtin -nostdinc -c advanced.c -o kimOS.o
	ld -m elf_i386 -T kimOS.ld bootloader.o kimOS.o -o kimOS.elf
mount:
	sudo losetup -Pf kimOS.img && \
	LOOP_DEVICE=$$(losetup -j kimOS.img | awk -F':' '{print $$1}'); \
	echo "Loop device is: $${LOOP_DEVICE}"; \
	sudo mount $${LOOP_DEVICE}p1 ./mnt
update:
	sudo cp ./kimOS.elf ./mnt/boot
umount:
	sudo umount ./mnt && \
	LOOP_DEVICE=$$(losetup -j kimOS.img | awk -F':' '{print $$1}'); \
	echo "Loop device is: $${LOOP_DEVICE}"; \
	sudo losetup -d $${LOOP_DEVICE}
clean:
	rm -f kimOS.elf kimOS.o bootloader.o 
