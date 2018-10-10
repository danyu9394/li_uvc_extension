# li_uvc_extension
  This is the sample code for __Leopard USB3.0__ camera for register control, PTS query 
  under Linux using V4L2. For supporting more UVC extension unit features, 
  firmware will need to get updated.

## hardware support
LI-AR0231-AP020X-GMSL2 USB3.0 camera
LI-OS05A20-MIPI-650IR V1.2 USB3.0 camera
LI-OV2311-MIPI-850IR V1.0 USB3.0 camera

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
make
```
- run the code
```
./uvc_extension_extra
```
- sequence of using save regsiter setting after power cycle
    1. put the register you want to save in __ChangConfigFromFlash__ and save
    2. compile and run the program with __load_register_setting_from_configuration__ _uncommented_
    3. power cycle the board(unplug usb&12V power supply)
    4. power on the board 
    5. run any camera streaming application in linux(e.g. guvcview)**MUST DO**
    6. _comment out_ __load_register_setting_from_configuration__, compile the code
    7. re-run the program, and your saved register info is loaded 


