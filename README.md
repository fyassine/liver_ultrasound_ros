## To start recording
1. Launch the camera driver:
`roslaunch azure_kinect_ros_driver driver.launch`
2. Launch the ultrasound video grabber:
`roslaunch ultrasound_capture capture.launch`
3. Start the recording launch file:
With Rviz:
`roslaunch sensor_dataset_recorder record_inside_rviz.launch`
when you record inside rviz, you will get a separate window where you have to press the `Enter Key`
Without Rviz:
`roslaunch sensor_dataset_recorder record.launch`
