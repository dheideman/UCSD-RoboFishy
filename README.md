# UCSD-RoboFishy

Raspberry Pi Zero Preparation
------
### Operating System
Raspbian Jessie Lite

### Enabling SSH over USB
From https://www.youtube.com/watch?v=O4oVIsZJDs4&t=248s:

Add to the end of config.txt:
  dtoverlay=dwc2
  
Add after "rootwait" in cmdline.txt:
  modules-load=dwc2,g_ether

Create file (no extension):
  ssh


### Parameters

#### [Camera Specifications](http://elinux.org/Rpi_Camera_Module)
Max Resolution of Camera:
3280 x 2464

Field of View of Camera:
62.2 x 48.8 degrees

Distance to Bottom:
Max 2.4 meters (for 1 mm resolution at edge of image)


Width x Height per picture at 2.4 meters from bottom:
2.9 x 2.18 meters

