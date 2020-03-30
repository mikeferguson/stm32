# STM32 Sandbox
This is a sandbox full of stuff/garbage for the STM32, specifically the STM32F4 series.

This repository is currently using:
 * CMSIS v3.01
 * STM32F4xx_StdPeriph_driver v1.0.1

Things to keep in mind if adapting to STM32F1, F2, or F3:
 * You'll need to locate the proper device-specific files (stored in libraries/ST folder).
 * Much of libcpp has been tested with F1 series. Notably, the gpio.h will not
   work with F2 or F3 processors right now.

# Setup of development environment (18.04)

I'm using the 4.7-2014-q2-update revision from the official GCC ARM launchpad
repository:  https://launchpad.net/gcc-arm-embedded

```
sudo apt-get install gcc-multilib libncurses5:i386

cd ~/bin
wget https://launchpad.net/gcc-arm-embedded/4.7/4.7-2014-q2-update/+download/gcc-arm-none-eabi-4_7-2014q2-20140408-linux.tar.bz2
tar xvfj gcc-arm-none-eabi-4_7-2014q2-20140408-linux.tar.bz2
rm gcc-arm-none-eabi-4_7-2014q2-20140408-linux.tar.bz2
echo 'export PATH=$PATH:~/bin/gcc-arm-none-eabi-4_7-2014q2/bin:$PATH' >> ~/.bashrc
```

## Build OpenOCD
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

## Build OpenOCD using libftdi
I previously used OpenOCD 0.8.0-rc2 and the libftdi interface:
```
sudo apt-get install libftdi-dev
cd ~/bin
git clone git://github.com/mikeferguson/openocd.git
git checkout v0.8.0-rc2
cd openocd
./bootstrap
./configure --enable-legacy-ft2232_libftdi
make
echo 'export PATH=$PATH:~/bin/openocd/src' >> ~/.bashrc
```

When using the libftdi interface, you would use interface/flyswatter2.cfg:

```
cd ~/bin/openocd/tcl
sudo ../src/openocd  -f interface/flyswatter2.cfg -f target/stm32f4x.cfg
```
