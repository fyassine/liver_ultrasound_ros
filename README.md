Camera Calibration

# IMPORTANT!! for both camera setup follow these steps:

1. Derivers and settings
- Install K4a driver for kinect:
<img width="2894" height="1472" alt="image" src="https://github.com/user-attachments/assets/8b71465d-d046-4f4d-9585-5a2cf3a8412f" />
- Update GRUB usbfs limit:
(https://github.com/microsoft/Azure_Kinect_ROS_Driver/issues/97):

By default, Linux limits image capturing to a Max_value or 16 MB.
In ( /etc/default/grub ), change ( GRUB_CMDLINE_LINUX_DEFAULT="quiet splash" ) to ==> ( GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000" ),
Update the grup ( sudo update-grub ) and restart your PC ( sudo reboot ). Check ( cat /sys/module/usbcore/parameters/usbfs_memory_mb ) that it shows the new value.

2. Calibrate both cameras separately:
- To find the correct serial number of eob or eih, use: k4aviewer (to view the camera video output) or also k4arecorder --list commands
- Then, for example: roslaunch easy_handeye publish_eob_inside_moveit.launch sensor_sn:=000187504512 
- Calibration files are stored under: /home/aorta-scan/.ros/easy_handeye/
but the ones that are correct for the current setup are extra in: /home/aorta-scan/fyassine/auto_liver_ultrasound/catkin_ws/src/easy_handeye/easy_handeye/launch/calib_files

3. Test cameras calibration inside moveit:
roslaunch easy_handeye publish_dual_inside_moveit.launch enable_eob:=true enable_eih:=true
