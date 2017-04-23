# UCSD-RoboFishy

Raspberry Pi Zero Preparation
------
### Operating System
Raspbian Jessie Lite

### Enabling SSH over USB
From https://www.youtube.com/watch?v=O4oVIsZJDs4&t=248s:

In the boot folder:

Add to the end of config.txt:
  `dtoverlay=dwc2`
  
Add after "rootwait" in cmdline.txt:
  `modules-load=dwc2,g_ether`

Create file (no extension):
  ssh

### Setting Static IP Address
In /etc/dhcpcd.conf:
  change the addresses for the interface you want to use
  * usb0  = USB connection (connecting to most computers)
  * wlan0 = WiFi connection
  * eth0  = Ethernet connection (not available on Pi Zero)
  
  Example:
```
interface usb0
  static ip_address=192.168.7.2/24
  static routers=192.168.7.1
  static domain_name_servers=192.168.7.1
```

  Note: the "/24" at the end of the "ip_address" is required for USB connections "routers" should be the address of the computer that you are connecting from "domain_name_servers" can either be the computer you are connecting from, or an actual domain name server like "8.8.8.8"

Parameters
------

#### [Camera Specifications](http://elinux.org/Rpi_Camera_Module)
Max Resolution of Camera:
3280 x 2464

Field of View of Camera:
62.2 x 48.8 degrees

Distance to Bottom:
Max 2.4 meters (for 1 mm resolution at edge of image)


Width x Height per picture at 2.4 meters from bottom:
2.9 x 2.18 meters

