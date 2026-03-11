# iiwa Robot Setup

Connecting Robot to PC and PC to Internet:

top port --> robot ethernet cable

bottom port --> ethernet (see settings)

Beware of bandwith limitation of two kinect cameras running simultaneously. 
Connect each two cables of each cameras separately (2 at the top PCIe and 2 at the bottom)

---

## iiwa Robot Joint Positions

### Connecting Robot to PC and PC to Internet
* **Top Port:** Robot ethernet cable
* **Bottom Port:** Ethernet (see settings)

![Robot Port Connection](https://github.com/user-attachments/assets/8fc018a4-5702-4908-aba6-8188a7d7ac3d)
![Network Settings](https://github.com/user-attachments/assets/2c043130-ce78-4c95-8d98-d15a85aa4081)

**To record a trajectory with joint positions using the `iiwa_msgs` format:**
```bash
roslaunch iiwa_trajectory record_joint_positions.launch

```

**To replay the trajectory:**

```bash
roslaunch iiwa_trajectory replay_joint_positions.launch bag:=/home/aorta-scan/fyassine/auto_liver_ultrasound/data/recorded_joint_positions_iiwa.bag

```

---

## Camera Calibration

### 1. Drivers and Settings

* **Install K4a driver for Kinect:**
* **Update GRUB usbfs limit:** *(Reference: [Azure Kinect ROS Driver Issue #97](https://github.com/microsoft/Azure_Kinect_ROS_Driver/issues/97))* By default, Linux limits image capturing to a maximum of 16 MB. To increase this:
1. Open `/etc/default/grub`.
2. Change `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"` to:
```bash
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"

```


3. Update GRUB: `sudo update-grub`
4. Restart your PC: `sudo reboot`
5. Verify the change by running: `cat /sys/module/usbcore/parameters/usbfs_memory_mb` (it should return `1000`).



> **Note:** Beware of bandwidth limitations when running two Kinect cameras simultaneously. Connect the two cables of each camera to separate USB buses (e.g., 2 at the top PCIe and 2 at the bottom).

### 2. Calibrate Cameras Separately

*(More details at: [IFL_calibration GitHub](https://github.com/Chiararipiemon/IFL_calibration))*

* **Find Serial Numbers:** To find the correct serial number for **EOB** (eye-on-base) or **EIH** (eye-in-hand), use the `k4aviewer` GUI or run:
```bash
k4arecorder --list

```


* **Launch Calibration:** Use the serial number in the launch command:
```bash
roslaunch easy_handeye publish_eob_inside_moveit.launch sensor_sn:=000187504512

```


* **File Locations:**
* **Default storage:** `/home/aorta-scan/.ros/easy_handeye/`
* **Current setup files:** `/home/aorta-scan/fyassine/auto_liver_ultrasound/catkin_ws/src/easy_handeye/easy_handeye/launch/calib_files`



### 3. Test Calibration in MoveIt

Run the following command to verify the dual camera setup:

```bash
roslaunch easy_handeye publish_dual_inside_moveit.launch enable_eob:=true eih_serial_no:=000188401612 eob_serial_no:=000187504512

```

---

## Microphone

Run the following command to record audio from the default microphone configured in your system settings:

```bash
roslaunch rear_mic_recorder record_rear_mic.launch

```

---

## Clarius Ultrasound Probe

* **Password:** `msot`
* **Network Setup:** Enable "Prefer Wireless LAN" and under "Clarius Cast" select "Research (5828)".

When launching the Clarius probe node, remember to export this path first:

Use password "msot"
Enable "Prefer Wireless LAN" and under "Clarius Cast" select "Research (5828)"
When launching the clarius probe node, remember to run:
```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7
```

```

**To record the probe video stream at 10 fps run:**

```bash
rostopic pub -r 10 /clarius/request_image std_msgs/Empty '{}'

```

---

-----------------------------------

# Record the MCAP dataset

In separate terminals, run the following commands in order:

```bash
roscore

```

```bash
roslaunch easy_handeye publish_dual_inside_moveit.launch enable_eob:=true eih_serial_no:=000188401612 eob_serial_no:=000187504512
roslaunch capture_clarius_ultrasound publish_probe_tip_tf.launch
rosrun capture_clarius_ultrasound clarius_driver_node.py --ip 10.23.0.73   --port 5828 
rostopic pub -r 10 /clarius/request_image std_msgs/Empty '{}'

```

Check inside RViz if the following nodes are publishing correctly:

* `/base_camera/rgb/image_raw`
* `/hand_camera/rgb/image_raw`
* `/clarius/bmode`

After verifying, start the recording:

```bash
roslaunch mcap_dataset_recorder record_mcap_dataset.launch

```

**The MCAP file contains:**

* `/base_camera/rgb/image_raw`
* `/base_camera/depth_to_rgb/image_raw`
* `/hand_camera/rgb/image_raw`
* `/hand_camera/depth_to_rgb/image_raw`
* `/clarius/bmode`
* `/iiwa/state/JointPosition`
* `/iiwa/state/JointPosition_standard`

Run the following to record the audio from the default microphone in a separate file:

What is missing:
- Transformation from the ultrasound image to the pose of the robot
- Distance between end-effector and center of the aruco marker
- 

```bash
roslaunch rear_mic_recorder record_rear_mic.launch

```
