SDCARD=/media/ce905953-c26e-411b-8047-9c1336a605c3
FEXC=../../github_linux-sunxi/sunxi-tools/fexc

echo "building modules tree..."
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- INSTALL_MOD_PATH=output modules_install

echo "deleting old tree from the sdcard..."
sudo rm $SDCARD/lib/modules/3.4.61-rt77-ARCH+/ -rf

echo "installing the new one..."
sudo cp output/lib/modules/3.4.61-rt77-ARCH+/ $SDCARD/lib/modules/ -r

echo "installing the kernel image"
sudo cp arch/arm/boot/uImage $SDCARD/boot/

echo "compiling script.bin..."
$FEXC script.fex script.bin

echo "copying it to the SD card..."
sudo cp script.bin $SDCARD/boot/

echo "copying MOD scripts to the SD card..."
sudo cp mod_scripts/* $SDCARD/root/

echo "unmounting the SD card..."
umount $SDCARD

echo "DONE!"
