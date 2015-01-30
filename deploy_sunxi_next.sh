VERSION=3.17.0-rc3-156746-g6c39f4e
SDCARD=/media/fsanches/ce905953-c26e-411b-8047-9c1336a605c3
FEXC=../../github_linux-sunxi/sunxi-tools/fexc

echo "building modules tree..."
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- INSTALL_MOD_PATH=output modules_install

echo "deleting old tree from the sdcard..."
sudo rm $SDCARD/lib/modules/$VERSION/ -rf

echo "installing the new one..."
sudo cp output/lib/modules/$VERSION/ $SDCARD/lib/modules/ -r

echo "installing the kernel image"
sudo cp arch/arm/boot/uImage $SDCARD/boot/

echo "copying DTB to the SD card..."
sudo mkdir -p $SDCARD/boot/dtbs/
sudo cp arch/arm/boot/dts/sun7i-a20-modduo.dtb $SDCARD/boot/dtbs/

echo "unmounting the SD card..."
umount $SDCARD

echo "DONE!"
