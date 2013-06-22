
The Akafugu Nixie Clock
------------------------

Firmware for The Akafugu Nixie Clock.

The Akafugu Nixie Clock is a fun to build stylish clock kit that uses old-fashioned neon Nixie tubes.

See the [product page](http://www.akafugu.jp/posts/products/nixie/) for more information.

Compiling
---------

There are two revisions of The Akafugu Nixie Clock available.
Version 1 and version 2.

Before compiling, you must set the type of clock you have by editing
global.h

Look in the beginning of the file:

// define the type of board we are compiling for
// The Akafugu Nixie Clock (http://www.akafugu.jp/posts/products/nixie/)
//#define BOARD_STANDARD
// Diet Nixie board (http://www.akafugu.jp/posts/products/diet_nixie/)
//#define BOARD_DIET
// The Akafugu Nixie Clock mk2 (http://www.akafugu.jp/posts/products/nixie/)
//#define BOARD_MK2

Uncomment the #define line containing the board you are compiling for.

Programming
----------

To reprogram the device, remove the display board and insert an FTDI adapter
into the header on the left hand side of the control board (marked GNC, CTS, VCC, TXO, RXI, DTR).
You will need to remove the enclosure to access the header. To use the header without removing
the Display board, you will need to attach jumper cables or an angled female header with long legs
first.

The microcontroller comes with a built-in bootloader for 8MHz internal clock.

If you are using the Akafugu Arduino package, you can select "Akafugu Nixie Clock",
otherwise, open your hardware/arduino/boards.txt file (found inside the directory where you installed
Arduino, or on OS X by selecting "Show Package Contents".

Add the following to the end of boards.txt

<code>
##############################################################

nixieclock.name=Akafugu Nixie Clock

nixieclock.upload.protocol=arduino
nixieclock.upload.maximum_size=30720
nixieclock.upload.speed=57600

nixieclock.bootloader.low_fuses=0xe2
nixieclock.bootloader.high_fuses=0xD8
nixieclock.bootloader.extended_fuses=0x05
nixieclock.bootloader.path=atmega
nixieclock.bootloader.file=ATmegaBOOT_168_atmega328_pro_8MHz.hex
nixieclock.bootloader.unlock_bits=0x3F
nixieclock.bootloader.lock_bits=0x0F

nixieclock.build.mcu=atmega328p
nixieclock.build.f_cpu=8000000L
nixieclock.build.core=arduino
nixieclock.build.variant=standard
</code>

Now select "Akafugu Nixie Clock" in the Boards menu.
