respeaker_ros
=============

A ROS Package for Respeaker Mic Array


## Supported Devices

- [Respeaker Mic Array v2.0](http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)

    ![Respeaker Mic Array v2.0](https://github.com/SeeedDocument/ReSpeaker_Mic_Array_V2/raw/master/img/Hardware%20Overview.png)

## Preparation

1. Install this package

    ```bash
    mkdir -p ~/catkin_ws/src && ~/catkin_ws/src
    git clone https://github.com/furushchev/respeaker_ros.git
    cd ~/catkin_ws
    source /opt/ros/kinetic/setup.bash
    rosdep install --from-paths src -i -r -n -y
    catkin config --init
    catkin build respeaker_ros
    source ~/catkin_ws/devel/setup.bash
    ```

1. Register respeaker udev rules

    Normally, we cannot access USB device without permission from user space.
    Using `udev`, we can give the right permission on only respeaker device automatically.

    Please run the command as followings to install setting file:

    ```bash
    roscd respeaker_ros
    sudo cp -f $(rospack find respeaker_ros)/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
    sudo systemctl restart udev
    ```

    And then re-connect the device.

1. Install python requirements

    ```bash
    roscd respeaker_ros
    sudo pip install -r requirements.txt
    ```

1. Update firmware

    ```bash
    git clone https://github.com/respeaker/usb_4_mic_array.git
    cd usb_4_mic_array
    sudo python dfu.py --download 6_channels_firmware.bin  # The 6 channels version 
    ```

## How to use

1. Run executables

    ```bash
    roslaunch respeaker_ros respeaker.launch
    rostopic echo /sound_direction     # Result of DoA
    rostopic echo /sound_localization  # Result of DoA as Pose
    rostopic echo /is_speeching        # Result of VAD
    rostopic echo /audio               # Raw audio
    rostopic echo /speech_audio        # Audio data while speeching
    ```

    You can also set various parameters via `dynamic_reconfigure`.

    ```bash
    sudo apt install ros-kinetic-rqt-reconfigure  # Install if not
    rosrun rqt_reconfigure rqt_reconfigure
    ```

    To set LED color, publish desired color:

    ```bash
    rostopic pub /status_led std_msgs/ColorRGBA "r: 0.0
    g: 0.0
    b: 1.0
    a: 0.3"
    ```

## Use cases

### Voice Recognition

- [ros_speech_recognition](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/ros_speech_recognition)
- [julius_ros](http://wiki.ros.org/julius_ros)

## Notes

The configuration file for `dynamic_reconfigure` in this package is created automatically by reading the parameters from devices.
Though it will be rare case, the configuration file can be updated as followings:

1. Connect the device to the computer.
1. Run the generator script.

    ```bash
    rosrun  respeaker_ros respeaker_gencfg.py
    ```
1. You will see the updated configuration file at `$(rospack find respeaker_ros)/cfg/Respeaker.cfg`.


## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>

## License

[Apache License](LICENSE)
