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
