Changing wifi connection
------
In /etc/wpa_supplicant/wpa_supplicant.conf:
  uncomment the lines below the label for the network to you wish to connect.


Changing Static IP Address
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
  
    the "/24" at the end of the "ip_address" is required for USB connections
    "routers" should be the address of the computer that you are connecting from
    "domain_name_servers" can either be the computer you are connecting from, or
      an actual domain name server like "8.8.8.8"


Switch Between Access Point and Connecting to WiFi
------
###### AP to Existing WiFi
In /etc/network/interfaces:
  uncomment line:
```
#   wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
```
  
Run commands:
```
sudo systemctl disable hostapd
sudo systemctl disable dnsmasq
```
  
###### Existing WiFi to AP
In /etc/network/interfaces:
  comment out line:
```
    wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
```
  
Run commands:
```
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
```