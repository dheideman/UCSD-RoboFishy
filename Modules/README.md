In order to shutdown Pi using a button press, 
1. Connect momentary switch between pins 39 & 40.
2. Add simple-shutdown.py to home folder
3. Insert the following into rc.local before the ```exit 0```
```
# python program that puts Pi in 'halt' state when pins 39 and 40 are shorted
# check out github.com/TonyLHansen/raspberry-pi-safe-off-switch for details
~pi/simple-shutdown.py
```

Note: gpiozero needs to be installed:
```
sudo apt-get install python-pip
pip install gpiozero --user
```
