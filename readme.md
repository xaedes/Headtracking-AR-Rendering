
# installation OpenNI2 in msys2
pacman -S mingw64/mingw-w64-x86_64-libusb

ln -s ~/OpenNI2/Lib/ /usr/lib/openni2
ln -s ~/OpenNI2/Include/ /usr/include/openni2
use FindOpenNI2.cmake


