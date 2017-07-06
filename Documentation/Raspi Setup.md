# Raspberry Pi Preparation (Zero, Zero W, or 3)

Operating System
------
Raspbian Jessie Lite


Install Image
------
Follow the directions for your host computer's operating system:
https://www.raspberrypi.org/documentation/installation/installing-images/


Enabling SSH over USB
------
https://www.thepolyglotdeveloper.com/2016/06/connect-raspberry-pi-zero-usb-cable-ssh/

1. In a terminal, navigate into the boot folder
2. Add to the end of config.txt:
   `dtoverlay=dwc2`
3. Add after "rootwait" in cmdline.txt:
   `modules-load=dwc2,g_ether`
4. Create ssh file (no extension) by running the command:
   `touch ssh`
5. In the file system, in /etc/dhcpcd.conf add:
   (More information on dhcpcd.conf below)
```
interface usb0
  static ip_address=192.168.7.2/24
  static routers=192.168.7.1
  static domain_name_servers=192.168.7.1
interface eth0
  static ip_address=192.168.7.2
  static routers=192.168.7.1
  static domain_name_servers=192.168.7.1
```
6. Exit, type `sync`, wait for it to finish, then take SD card out and put into pi
7. If using a Pi Zero, plug a USB cable into the computer and then into the USB port on the Pi (not the power port).  For all other Pis, use an ethernet cable instead.
8. Open a terminal and type `sudo ip a add 192.168.7.1/24 dev usb0` for a Pi Zero or `sudo ip a add 192.168.7.1/24 dev eth0` for all other Pis.
9. You're ready to ssh into that puppy! Type `ssh pi@192.168.7.2`, then type default pw: `raspberry`

Note:
  * Even if you do not plan to ssh into the Pi over USB, it is advisable to perform this step, as it also enables SSH over Ethernet.  Change `interface usb0` to `interface eth0`.


Connecting to WiFi
------
/etc/network/interfaces:
```
allow-hotplug wlan0
iface wlan0 inet dhcp
    wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
```

Generate Password Hash
```
echo -n password_here | iconv -t utf16le | openssl md4
```
    
/etc/wpa_supplicant/wpa_supplicant.conf:
```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
ctrl_interface_group=0
update_config=1
network={
  ssid="<your WiFi network name>"
  psk=hash:<your hash here>
}
```

Note:
  * The built-in Wi-Fi chips on the Pi 3 and Pi Zero do not support 5 GHz Wi-Fi connections.  Use only 2.4 GHz Wi-Fi networks.
  * See below for information on connecting to "eduroam" networks.
  * Multiple network connections can be added in wpa_supplicant.conf by adding another `network={...}` tag.
  * Priorities for each network can be set using the tag `priority=...` inside each `network` tag.
  
  
Setting Static IP Address
------
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

Notes:
  * The "/24" at the end of the "ip_address" is required for USB connections
  * "routers" should be the address of the computer that you are connecting from
  * "domain_name_servers" can either be the computer you are connecting from, or an actual domain name server like "8.8.8.8"
  * When connecting to your Pi over WiFi using a static IP address, it may be necessary to change in /etc/network/interfaces `iface wlan0 inet dhcp` to `iface wlan0 inet manual`.


Connecting to eduroam
------
https://servicedesk.rose-hulman.edu/link/portal/710/4769/Article/372/How-do-I-connect-a-Raspberry-Pi-to-the-Eduroam-Wireless-Network

/etc/network/interfaces:
```
allow-hotplug wlan0
iface wlan0 inet dhcp
    wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
```
    
/etc/wpa_supplicant/wpa_supplicant.conf:
```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
ctrl_interface_group=0
update_config=1
network={
   ssid="eduroam"
   key_mgmt=WPA-EAP
   scan_ssid=1
   ca_cert="/etc/ssl/certs/AddTrust_External_Root.pem"
   eap=PEAP
   phase2="auth=MSCHAPV2"
   identity="<your university email address>"
   password=hash:<your hash here>
}
```

Notes:
  * For UCSD, the login name for eduroam is your university email address (i.e. "robofishy@ucsd.edu").
  * For other institutions, use whatever login you usually use for eduroam.


Expand Filesystem
------
Before compiling OpenCV, expand the file system to take advantage of all the space on your SD card
```
sudo raspi-config
```
under advanced options, expand filesystem


Compile OpenCV
------

If `git` and `cmake` are not installed on your system (a brand-new Raspbian distro does not include them), install them with:
```bash
sudo apt-get install git
sudo apt-get install cmake
```

Get all the opencv dependencies:
```bash
sudo apt-get update
sudo apt-get install libopencv-dev
```


http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html


Download OpenCV from GitHub
```bash
cd ~/<my_working _directory>
git clone https://github.com/opencv/opencv.git
```

If you do not have internet access on your Pi, the OpenCV source code can be downloaded on a different computer and transferred with `scp`:
```bash
scp -r opencv pi@<ip address>:~/
```
Which will copy opencv to the home directory on the pi.

Once OpenCV is installed on your computer, compile it with:
```bash
cd ~/opencv
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..

make
sudo make install
```

Raspberry Pi Camera Module
------
If you are using a Raspberry Pi Camera Module, you will need to enable it.  Run
```
sudo raspi-config
```
and select "Interfacing Options", then select the first menu item, "Enable Camera".

This will require a reboot of the Pi.


V4L2 Driver for Raspberry Pi
------

For those using a Raspberry Pi Camera Module, someone said on the internet:

Using V4L2 driver for raspberry camera `sudo modprobe bcm2835-v4l2`, you could access it like USB camera.

http://raspberrypi.stackexchange.com/questions/28458/rpi-camera-module-with-opencv

Added `bcm2835-v4l2` to a new line in /etc/modules-load.d/modules.conf
(commented out 2017/4/20)
(uncommented 2017/4/29)

Settings:

https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=62364

```
# load the module (or add module name to /etc/modules-load.d/modules.conf (\n))
sudo modprobe bcm2835-v4l2

# viewfinder
v4l2-ctl --overlay=1 # enable viewfinder
v4l2-ctl --overlay=0 # disable viewfinder

# record video
v4l2-ctl --set-fmt-video=width=1920,height=1088,pixelformat=4
v4l2-ctl --stream-mmap=3 --stream-count=100 --stream-to=somefile.264

# capture jpeg: max resolution = 3280 × 2464
v4l2-ctl --set-fmt-video=width=3280,height=2464,pixelformat=3
v4l2-ctl --stream-mmap=3 --stream-count=1 --stream-to=somefile.jpg

# set bitrate
v4l2-ctl --set-ctrl video_bitrate=10000000

# list supported formats
v4l2-ctl --list-formats
```


RaspiCam
------
Not currently in use:  See V4L2 Driver above.

Raspicam allows for the adjustment of more raspberry pi camera properties than pure OpenCV.
https://www.uco.es/investiga/grupos/ava/node/40

```
cd raspicamxx
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

https://sourceforge.net/p/raspicam/tickets/2/
CMakeLists.txt:
```
cmake_minimum_required(VERSION 2.8)
project( ExposureTests )
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)
find_package( OpenCV )
find_package( raspicam REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( ExposureTests ExposureTests.cpp )
target_link_libraries( ExposureTests ${OpenCV_LIBS} ${raspicam_CV_LIBS} )
```


Create USB Flash Drive Mount Point
------
```
sudo mkdir /mnt/usb
```

Mount drive (need to run every time)
```
sudo mount /dev/sda1 /mnt/usb
```

May be able to mount on boot (needs to be the same flash drive every time):
http://www.techjawab.com/2013/06/how-to-setup-mount-auto-mount-usb-hard.html


Create Wireless Access Point
------
http://www.raspberryconnect.com/network/item/315-rpi3-auto-wifi-hotspot-if-no-internet


Configure RasPi for I2C
------
https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c

Set Hostname
------
In /etc/hostname, the hostname


Setup RasPi for BNO055 IMU
------
Disable bluetooth channel to enable UART connection

sudo nano /boot/config.txt

At the bottom of this file, insert the following lines and comment out dtoverlay=dwc2:
```
dtoverlay=pi3-disable-bt
enable_uart=1
```


Setup RasPi for DS18B20 temperature sensor:
------
https://pimylifeup.com/raspberry-pi-temperature-sensor/

Open up the boot config file:
``'
sudo nano /boot/config.txt
'''

At the bottom of this file, insert the following line:
```
dtoverlay=w1-gpio
```

Reboot the Raspi:
```
sudo reboot
```

Load the necessary modules:
```
sudo modprobe w1-gpio
sudo modprobe w1-thermo
```


Setup RasPi to run and kill python scripts from within c++ programs:
------
https://stackoverflow.com/questions/34687883/starting-stopping-a-background-python-process-wtihout-nohup-ps-aux-grep-kill
Add this to the end of your .bashrc file in your home folder:
```
# Added functions to start and kill python scripts
mynohup () {
    [[ "$1" = "" ]] && echo "usage: mynohup python_script" && return 0
    nohup python -u "$1" > "${1%.*}.log" 2>&1 < /dev/null &
}

mykill() {
    ps -ef | grep "$1" | grep -v grep | awk '{print $2}' | xargs kill
    echo "process "$1" killed"
}
```
