````markdown
## PREREQUISITES

Install the following from Canvas:

- `qnx_sdp8.0_rpi4_quickstart_20250820.img` (Raspberry Pi 4 base image)
- `ros2_humble_qnx_aarch64.tar.gz` (base ROS 2 runtime for QNX)
- `demo` (contains `imu_pub` and `imu_sub` Python files)

---

## CONTENTS

- **SECTION A:** Imaging the Pi with QNX  
- **SECTION B:** Running the IMU Pub/Sub demo on QNX + ROS2  
- **Explanations**

---

## SECTION A: Imaging the Pi with QNX

### Step 1

Identify your SD card device (use the **WHOLE** device, not a partition):

```bash
lsblk -p -o NAME,MODEL,SIZE,MOUNTPOINT
````

Example to use: `/dev/sdc` (do **NOT** use `/dev/sdc1` or `/dev/sdc2`).

---

### Step 2

Flash the image (adjust paths and device):

```bash
sudo dd if=/home/yourname/qnx800/images/qnx_sdp8.0_rpi4_quickstart_20250820.img of=/dev/sdc bs=4M status=progress conv=fsync
```

---

### Step 3

In a new terminal:

```bash
cd /media/username/boot
```

Edit WiFi in `wpa_supplicant.conf`:

```conf
network={
    ssid="WIFI-Z"
    psk="password"
}
```

---

### Step 4

When all is done, run:

```bash
sync
sudo eject /dev/sdc
```

---

### Step 5

* Remove the SD card safely.
* Insert it into your Raspberry Pi and power it on.

After this, the Pi is imaged and ready for use.

---

## SECTION B: Running the IMU Pub/Sub demo on QNX + ROS2

**Objective:** Learn how to run your own Python files with QNX/ROS2 on the Pi.
**Exercise:** Publish real IMU readings on `/imu/QNX_1/data` and view them.

---

### Step 1

SSH into the Pi:

```bash
ping am-imu-01.local
ssh qnxuser@am-imu-01.local
```

Password is:

```text
qnxuser
```

---

### Step 2

Copy tarballs to the Pi (from your Linux computer):

```bash
scp ros2_humble.tar.gz qnxuser@am-imu-01.local:/data/home/qnxuser
scp demo.tar.gz        qnxuser@am-imu-01.local:/data/home/qnxuser
```

*(This might take ~6–10 minutes depending on SD card speed and network.)*

---

### Step 3

Back on the Pi, unpack the files we just downloaded:

```bash
cd /data/home/qnxuser
mkdir -p opt/ros/humble opt/ros/nodes demo
tar -xzvf ros2_humble.tar.gz -C opt/ros/humble --strip-components=1
tar -xzvf demo.tar.gz        -C demo --strip-components=1
```

---

### Step 4

Run the demo with two terminals.

**Terminal 1 (publisher):**

```bash
. opt/ros/humble/setup.bash
/system/bin/python3 ~/demo/imu_pub.py
```

**Terminal 2 (subscriber):**

```bash
. opt/ros/humble/setup.bash
/system/bin/python3 ~/demo/imu_sub.py
```

You should see roll/pitch/yaw numbers streaming.
If your IMU outputs different fields (e.g., accel/gyro), adjust the parsing line in `imu_serial_pub.py` to map the correct column.

---

## Explanations

There are 3 main layers in our setup:

1. The **QNX layer**
2. The **ROS2 layer**
3. The **Python scripts** with our demo

### QNX Layer

QNX is our base operating system which provides a **real-time** OS on the Raspberry Pi.
Linux is not a real-time system. This means QNX guarantees that critical tasks like sensor polling or control loops happen within precise time limits. It does this through:

1. **Microkernel architecture**
   Only the essential kernel runs in privileged mode; everything else (drivers, filesystems, networking) runs as separate user-space processes.

2. **Priority-based scheduling**
   Higher-priority tasks always run first, which avoids latency spikes.

3. **Deterministic message passing**
   Inter-process communication is predictable and bounded in time.

Essentially, where Linux might pause during heavy I/O or context switches, QNX ensures that real-time tasks run within their timing requirements.

### ROS2 Layer

ROS2 Humble is our middleware layer that provides modular messaging and communication capabilities between processes (these are called **nodes**).

If you see the acronym **DDS** (Data Distribution Service), this refers to the standard protocol used for all messaging, which operates in a **publish/subscribe** fashion.

### Demo Python Scripts

Our demo (`imu_pub.py` and `imu_sub.py`) contains the application nodes using:

* `rclpy` (the Python client library for ROS2)
* `sensor_msgs` (ROS2 message definitions for sensors)

The Raspberry Pi boots QNX as the operating system. The image we flashed contains:

* QNX kernel
* Drivers
* Shell
* Python runtime

All of the paths we created for the demo and ROS files exist in QNX's filesystem.

The `ros2_humble.tar.gz` file is the QNX-compiled build of ROS2 Humble.
When we extract and source it, we are setting up environment variables that point to the QNX-compatible binaries and libraries.

This means that when we run the Python scripts, we are importing packages from the **QNX-built** ROS2 installation, not a normal Linux one.

* `imu_pub` creates and publishes a `sensor_msgs/msg/Imu` message.
* `imu_sub` subscribes to that topic via DDS and logs the IMU data.

```
```

