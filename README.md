PREREQUISITES

Install the following from canvas:

- qnx_sdp8.0_rpi4_quickstart_20250820.img (raspberry pi 4 base image)
- ros2_humble_qnx_aarch64.tar.gz (base ROS 2 runtime for QNX)
- demo (contains imu_pub and imu_sub python files)

--------------------------------------------------------------------------
CONTENTS

SECTION A: Imaging the Pi with QNX
SECTION B: Running the IMU Pub/Sub demo on QNX + ROS2
Explanations

--------------------------------------------------------------------------
SECTION A: Imaging the Pi with QNX

Step 1:

Identify your SD card device (use the WHOLE device, not a partition):
lsblk -p -o NAME,MODEL,SIZE,MOUNTPOINT
Example to use: /dev/sdc (do NOT use /dev/sdc1 or /dev/sdc2)

Step 2:

Flash the image (adjust paths and device):
sudo dd if=/home/yourname/qnx800/images/qnx_sdp8.0_rpi4_quickstart_20250820.img of=/dev/sdc bs=4M status=progress conv=fsync

Step 3: 

in a new terminal , cd into /media/username/boot 

change wifi in for wpa_supplicant.conf

network={
    ssid="WIFI-Z"
    psk="password"
}


Step 4:

When all is done, run:
sync
sudo eject /dev/sdc


Step 5: 

Remove the SD card safely.
Insert it into your Raspberry Pi and power it on.

After this, the Pi is imaged and ready for use.

--------------------------------------------------------------------------
SECTION B: Running the IMU Pub/Sub demo on QNX + ROS2
Objective: Learn how to run your own python files with qnx/ros2 on the pi
Exercise: publishing real IMU readings on /imu/QNX_1/data and viewing them

Step 1. 

ssh into the pi
ping am-imu-01.local
ssh qnxuser@am-imu-01.local

password is qnxuser

Step 2: 

Copy tarballs to the Pi (from your linux computer)
scp ros2_humble.tar.gz qnxuser@am-imu-01.local:/data/home/qnxuser (this might take like 6-10 minutes depending on the card speed and network)
scp demo.tar.gz qnxuser@am-imu-01.local:/data/home/qnxuser


Step 3: 
Back on the Pi, unpack the files we just downloaded
cd /data/home/qnxuser
mkdir -p opt/ros/humble opt/ros/nodes demo
tar -xzvf ros2_humble.tar.gz -C opt/ros/humble --strip-components=1
tar -xzvf demo.tar.gz        -C demo --strip-components=1


Step 4:
Run the demo with two terminals. 

In terminal 1 (the publisher):
. opt/ros/humble/setup.bash
/system/bin/python3 ~/demo/imu_pub.py

In terminal 2 (the subscriber):
. opt/ros/humble/setup.bash
/system/bin/python3 ~/demo/imu_sub.py

You should see roll/pitch/yaw numbers streaming. If your IMU outputs different fields (e.g., accel/gyro), adjust the parsing line in imu_serial_pub.py to map the correct column
------------------------------------------------------------------------------------------------

Explanations:

There are 3 main layers in our setup - the QNX layer, the ROS2 layer, and the python scripts with our demo.

QNX is our base operating system which provides a real-time operating system on the raspberry pi. Linux is not a real time system. This means that QNX guarantees that critical tasks like sensor polling or control loops happen in precise time limits. It does this through:

1. A microkernel architecture (only the essential kernel runs in privileged mode, everything else like drivers/filesystems/networking run as separate user space processes
2. Priority based schedule means that higher priority tasks always run first which avoids latency spikes
3. Deterministic message passing whihc means that inter process communication is predictable and bounded in time. 

Essentailly where Linux might pause during heavy I/O or context switches, QNX ensures that real time tasks run within their time requirements.

ROS2 humble is our middleware layer that provides modular messaging and communication capabilities between processes (these are known as nodes). If you see the acronym DDS (data distribution service), this refers to the standarrd protocol used for all messaging which operates in a publish/subscribe fashion.

Our demo (imu_pub.py and imu_sub.py) contains the application nodes using rclpy (the python library for ros2) and sensor_msgs.

The Raspberry pi boots QNX as the operating system.  The image we flashed contains the QNX kernel, drivers, shell, and python runtime. All of the paths we created for the demo and ros files exist in QNX's filesystem.

The ros2_humble.tar.gz file is the qnx compiled build of ros2 humble. When we extract and source it we are setting up the environment variables that point to the qnx compatible binaries and llibraries. 

This means that when we run the python scripts, we are importing packages from the QNX-build ROS2 installtion, not the normal linux one.

imu_pub creates and publishes a sensor_msgs/msg/imu message
imu_sub subscribes to that topic via dds.
