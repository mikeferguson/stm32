#STM32 Sandbox
This is a sandbox full of stuff/garbage for the STM32.

This repository is currently using:
 * CMSIS v2.10, although the directory structure has been modified to be more like CMSIS v3.00. 
 * STM32F4xx_StdPeriph_driver, v1.0.1

#Setup of development environment (12.04)

## Grab gcc-based toolchain
I'm using the 4.6-2012-q2-update revision from the official GCC ARM launchpad repository:  https://launchpad.net/gcc-arm-embedded

```
sudo apt-get install ia32-libs

cd ~/bin
wget https://launchpad.net/gcc-arm-embedded/4.6/4.6-2012-q2-update/+download/gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2
tar xvfj gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2
rm gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2.tar.bz2
echo 'export PATH=$PATH:~/bin/gcc-arm-none-eabi-4_6-2012q2/bin:$PATH' >> ~/.bashrc
```

## Build OpenOCD
```
sudo apt-get install libftdi-dev
cd ~/bin
git clone git://github.com/mikeferguson/openocd.git
cd openocd
./bootstrap
./configure --enable-ft2232_libftdi
make
echo 'export PATH=$PATH:~/bin/openocd/src' >> ~/.bashrc
```

## Set paths

# Connecting to a Target
So far I haven't sorted out why openocd hates me, but the following command
works around issues with jimtcl paths:

```
cd ~/bin/openocd/tcl
sudo ../src/openocd  -f interface/flyswatter2.cfg -f target/stm32f4x.cfg
```

Sudo may or may not be neccessary depending on your group configurations.

