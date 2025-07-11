# Radiacode Driver for ROS 2

## Overview

This ROS 2 package provides an interface for reading data from Radiacode 10x series devices, based on the [cdump/radiacode](https://github.com/cdump/radiacode.git) project. It has been tested on the Radiacode 103.

## Prerequisites

Before using this package, ensure you have the required dependencies installed:

```bash
sudo apt install -y libglib2.0-dev
pip install radiacode
```

Additionally, set up the necessary udev rules for USB access:

```bash
sudo cp radiacode.rules /etc/udev/rules.d/99-radiacode.rules
```

**Udev configuration on the jetson:**
```bash
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="f123", ENV{DEVTYPE}=="usb_device", MODE="0666", TAG+="uaccess"
```
**If you encounter an "access denied" error when running the driver, do the following commands, or reboot your system to apply the changes :**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```


## Building the Package

To build the package, navigate to your ROS 2 workspace and run:

```bash
colcon build --symlink-install
```

## Running the Driver

Before launching the driver, source the workspace:

```bash
source install/setup.sh
```

### Default Launch

To start the driver with default settings:

```bash
ros2 launch radiacode_driver radiacode_driver.launch.py
```

### USB Mode

To explicitly launch the driver in USB mode:

```bash
ros2 launch radiacode_driver radiacode_driver.usb.launch.py
```

### Bluetooth Mode

To explicitly launch the driver in Bluetooth mode:

```bash
ros2 launch radiacode_driver radiacode_driver.bluetooth.launch.py
```

## Message structure

| Topic       | Type                                                                         |
| ----------- | ---------------------------------------------------------------------------- |
| /spectrum   | [Spectrum](/msg/Spectrum.msg)                                                |
| /dose_rate  | [Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) |
| /count_rate | [Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) |

## Additional Information

- This package is compatible with **ROS 2 Humble**.
- Contributions and improvements are welcome!
- For inquiries, contact **<capra@ens.etsmtl.ca>**.

For more details on the underlying Radiacode library, visit the [original repository](https://github.com/cdump/radiacode.git).
