run:
	qemu-system-i386 -hda kimOS.img
kernel:
	as --32 bootloader.S -o bootloader.o
	gcc -m32 -fno-stack-protector -fno-builtin -nostdinc -c advanced.c -o kimOS.o
	ld -m elf_i386 -T kimOS.ld bootloader.o kimOS.o -o kimOS.elf
mount:
	sudo mount /dev/loop23p1 ./mnt
update:
	sudo cp ./kimOS.elf ./mnt/boot
umount:
	sudo umount ./mnt
clean:
	rm -f kimOS.elf kimOS.o bootloader.o
