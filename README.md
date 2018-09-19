# li_uvc_extension
  This is the sample code for __Leopard USB3.0 AR0231 AP0200 GMSL2__ camera for 
  register control under Linux using V4L2. For supporting more UVC extension
  unit features, firmware will need to get updated.

## hardware support
AR0231 AP020X GMSL2 USB3.0 camera

## test environment
- kernel: 4.15.0-34-generic 
- distro version: Ubuntu 16.04.5 LTS

## how to run code
- install dependencies
```
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install v4l-utils
```
- compile the code
```
gcc -o test uvc_extension_extra.c
```
- run the code
```
./test
```
