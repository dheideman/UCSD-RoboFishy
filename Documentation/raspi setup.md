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
   identity="<your login name here>"
   password=hash:<your hash here>
}
```

Generate Password Hash
```
echo -n password_here | iconv -t utf16le | openssl md4
```


Get all the opencv dependencies
------
```
sudo apt-get update
sudo apt-get install libopencv-dev
```


Compile OpenCV
------
http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html

Expand Filesystem
------
Before compiling OpenCV, expand the file system to take advantage of all the space on your SD card
```
sudo raspi-config
```
under advanced options, expand filesystem

If `git` and `cmake` are not installed on your system (a brand-new Raspbian distro does not include them), install them with:
```bash
sudo apt-get install git
sudo apt-get install cmake
```

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


V4L2 Driver for Raspberry Pi
------
Currently not using V4L2 driver: see RaspiCam

Someone said on the internet:

Using V4L2 driver for raspberry camera `sudo modprobe bcm2835-v4l2`, you could access it like USB camera.

http://raspberrypi.stackexchange.com/questions/28458/rpi-camera-module-with-opencv

Added `bcm2835-v4l2` to a new line in /etc/modules-load.d/modules.conf
(commented out 2017/4/20)

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
Using raspicam because it allows for the adjustment of more raspberry pi camera properties.
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
