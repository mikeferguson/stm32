#STM32 Sandbox
This is a sandbox full of stuff/garbage for the STM32.

#Setup of development environment (Ubuntu 10.04/12.04)

## Grab gcc-based toolchain
* TODO

## Build OpenOCD
```
sudo apt-get install libftdi-dev
cd ~/bin
git clone git://github.com/mikeferguson/openocd.git
cd openocd
./bootstrap
./configure --enable-ft2232_libftdi
make
```

## Set paths
`echo 'export PATH=$PATH:~/bin/arm/bin:~/bin/openocd/src' >> ~/.bashrc`

# Connecting to a Target
So far I haven't sorted out why openocd hates me, but the following command
works around issues with jimtcl paths:

```
cd ~/bin/openocd/tcl
sudo ../src/openocd  -f interface/flyswatter2.cfg -f target/stm32f4x.cfg
```

Sudo may or may not be neccessary depending on your group configurations.

