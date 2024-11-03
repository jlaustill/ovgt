# ovgt
Open Source VGT controller, lets getter done!

## Setup

### STlink and udev
You will need to have an STLINK-V3 and the proper udev rule

``` bash
sudo nano /etc/udev/rules.d/49-stlinkv3.rules
```

```
# Rule for ST-LINK V3 for flashing
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3743", MODE="0666", GROUP="plugdev"

# Rule for serial access (/dev/ttyACM0) for monitoring
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="dialout"

```

idVendor and idProduct should match your ST-LINK device. Here, 0483:374e is standard for the ST-LINK V3, but you can confirm it by running lsusb and looking for your ST-LINK entry.

``` bash
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo usermod -aG plugdev $USER
```