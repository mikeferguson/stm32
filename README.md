# STM32 Sandbox
This is a sandbox full of stuff/garbage for the STM32, specifically the STM32F4 series.

This repository is currently using:
 * CMSIS v3.01
 * STM32F4xx_StdPeriph_driver v1.0.1

Things to keep in mind if adapting to STM32F1, F2, or F3:
 * You'll need to locate the proper device-specific files (stored in libraries/ST folder).
 * Much of libcpp has been tested with F1 series. Notably, the gpio.h will not
   work with F2 or F3 processors right now.

# Setup of development environment (24.04)

The toolchain had to be updated for 24.04 since the older gdb links against
libncurses5:i386, which is no longer available:

```
cd ~/bin
wget https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz
tar xf arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz
rm arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz
echo 'export PATH=$PATH:~/bin/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin:$PATH' >> ~/.bashrc
```

# Setup of development environment (22.04)

I'm using the 2020-q4-major revision from the official GCC ARM launchpad
repository:  https://launchpad.net/gcc-arm-embedded

```
sudo apt-get install gcc-multilib libncurses5:i386

cd ~/bin
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
tar xvfj gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
rm gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
echo 'export PATH=$PATH:~/bin/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH' >> ~/.bashrc
```

## Installing OpenOCD
```
sudo apt-get install openocd
```

## Build DSP_Lib (optional)
In the CMSIS directory, you can build the DSP_Lib by running Make. You may need
to change the target processor as it is currently M4lf (Cortex M4,
little-endians, with floating point).

# Connecting to a Target
So far I haven't sorted out why openocd hates me, but the following command
works around issues with jimtcl paths:

```
openocd  -f interface/ftdi/flyswatter2.cfg -f target/stm32f4x.cfg
openocd  -f interface/stlink.cfg -f target/stm32f4x.cfg
```

Sudo may or may not be neccessary depending on your group configurations.
Note: the board/stm32f4discovery.cfg file works well for ST-Link V2 and SWD.

I tend to use gdb to upload code and interact with the JTAG/STM32. The example
makefile has a ".gdbinit" target which exports a .gdbinit file that allows you
to run arm-none-eabi-gdb from within the project directory, and exposes a "flash"
command to upload firmware, and a "reset" command that works around some quirks
in either OpenOCD/Flyswatter/Lack-Of-Moon-Alignment:

```
cd <project>
arm-none-eabi-gdb
> flash
```

# Common Problems

## Undefined reference to '__aeabi_f2d', '__aeabi_d2iz', etc
The FPU is only single precision, using doubles cause lots of problems -- if
you see an error like this, make sure you are appending 'f' to floats:

    // 0.0 doesn't work, 0.0f does.
    float val = 0.0f;
